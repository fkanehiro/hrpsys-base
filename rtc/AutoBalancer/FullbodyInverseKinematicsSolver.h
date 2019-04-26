#ifndef FULLBODYIK_H
#define FULLBODYIK_H

#include <iomanip>
#include <hrpModel/JointPath.h>
#include "../ImpedanceController/RatsMatrix.h"
#include "../TorqueFilter/IIRFilter.h"
#include "MyUtils.h"
//#include <unsupported/Eigen/MatrixFunctions>
//#include <hrpUtil/MatrixSolvers.h>


#define OPENHRP_PACKAGE_VERSION_320

class IKConstraint {
    public:
        std::string target_link_name;
        hrp::Vector3 targetPos;
        hrp::Vector3 targetRpy;
        hrp::Vector3 localPos;
        hrp::Matrix33 localR;
        hrp::dvector6 constraint_weight;
        double pos_precision, rot_precision;
        IKConstraint ()
        :targetPos(hrp::Vector3::Zero()),
         targetRpy(hrp::Vector3::Zero()),
         localPos(hrp::Vector3::Zero()),
         localR(hrp::Matrix33::Identity()),
         constraint_weight(hrp::dvector6::Ones()),
         pos_precision(1e-4), rot_precision(deg2rad(0.1)){}
        hrp::Vector3 getCurrentTargetPos(hrp::BodyPtr _robot){ return _robot->link(target_link_name)->p + _robot->link(target_link_name)->R * localPos; }
        hrp::Matrix33 getCurrentTargetRot(hrp::BodyPtr _robot){ return _robot->link(target_link_name)->R * localR; }
};


class FullbodyInverseKinematicsSolver{
    protected:
        hrp::dmatrix J_all, J_all_inv, J_com, J_am;
        hrp::dvector dq_all, err_all, constraint_weight_all, jlim_avoid_weight_old;
        const int WS_DOF, BASE_DOF, J_DOF, ALL_DOF;
        std::vector<IIRFilter> am_filters;
        const double m_dt;
    public:
        hrp::BodyPtr m_robot;
        hrp::dvector dq_weight_all, q_ref, q_ref_max_dq, q_ref_constraint_weight;
        hrp::Vector3 cur_momentum_around_COM, cur_momentum_around_COM_filtered, rootlink_rpy_llimit, rootlink_rpy_ulimit;
        FullbodyInverseKinematicsSolver (hrp::BodyPtr _robot, const std::string& _print_str, const double _dt):
          m_robot(_robot),
          m_dt(_dt),
          WS_DOF(6),
          BASE_DOF(6),
          J_DOF(_robot->numJoints()),
          ALL_DOF(J_DOF+BASE_DOF){
            dq_weight_all = hrp::dvector::Ones(ALL_DOF);
            dq_all = q_ref = q_ref_constraint_weight = hrp::dvector::Zero(ALL_DOF);
            q_ref_max_dq = hrp::dvector::Constant(ALL_DOF, 1e-2);
            cur_momentum_around_COM = cur_momentum_around_COM_filtered = hrp::Vector3::Zero();
            jlim_avoid_weight_old = hrp::dvector::Constant(ALL_DOF, 0);
            rootlink_rpy_llimit = hrp::Vector3::Constant(-DBL_MAX);
            rootlink_rpy_ulimit = hrp::Vector3::Constant(DBL_MAX);
            am_filters.resize(3);
            for(int i=0;i<3;i++){
              am_filters[i].setParameterAsBiquad(10, 1/std::sqrt(2), 1.0/m_dt);
              am_filters[i].reset();
            }
        };
        ~FullbodyInverseKinematicsSolver () {};
        int numStates(){ return ALL_DOF; };
        bool checkIKConvergence(const std::vector<IKConstraint>& _ikc_list){
            for ( int i=0; i<_ikc_list.size(); i++ ) {
                hrp::Link* link_tgt_ptr = m_robot->link(_ikc_list[i].target_link_name);
                hrp::Vector3 pos_err, rot_err;
                if(link_tgt_ptr){
                    pos_err = _ikc_list[i].targetPos - (link_tgt_ptr->p + link_tgt_ptr->R * _ikc_list[i].localPos);
                    rats::difference_rotation(rot_err, (link_tgt_ptr->R * _ikc_list[i].localR), hrp::rotFromRpy(_ikc_list[i].targetRpy));
                }
                else if(!link_tgt_ptr && _ikc_list[i].target_link_name == "COM"){  // COM
                    pos_err = _ikc_list[i].targetPos - (m_robot->calcCM() + _ikc_list[i].localR * _ikc_list[i].localPos);
                    rot_err = _ikc_list[i].targetRpy - cur_momentum_around_COM;
                }
                else{ std::cerr<<"Unknown Link Target !!"<<std::endl; continue; }
                for(int j=0;j<3;j++){
                    if(_ikc_list[i].constraint_weight.head(3)(j) > 0 && fabs(pos_err(j)) > _ikc_list[i].pos_precision){return false;}
                    if(_ikc_list[i].constraint_weight.tail(3)(j) > 0 && fabs(rot_err(j)) > _ikc_list[i].rot_precision){return false;}
                }
            }
            return true;
        }
        int solveFullbodyIKLoop (const std::vector<IKConstraint>& _ikc_list, const int _max_iteration) {
            #ifdef OPENHRP_PACKAGE_VERSION_320
                hrp::Vector3 base_p_old = m_robot->rootLink()->p;
                hrp::Matrix33 base_R_old = m_robot->rootLink()->R;
                hrp::dvector q_old(J_DOF);
                for(int i=0;i<J_DOF;i++){ q_old(i) = m_robot->joint(i)->q; }
                unsigned int loop;
                for(loop=0; loop < _max_iteration;){
                    solveFullbodyIKOnce(_ikc_list);
                    //check ang moment
                    m_robot->rootLink()->v = (m_robot->rootLink()->p - base_p_old)/ m_dt;
                    m_robot->rootLink()->w = base_R_old * hrp::omegaFromRotEx(base_R_old.transpose() * m_robot->rootLink()->R) / m_dt;
                    for(int i=0;i<J_DOF;i++){ m_robot->joint(i)->dq = (m_robot->joint(i)->q - q_old(i)) / m_dt; }
                    m_robot->calcForwardKinematics(true,false);
                    hrp::Vector3 tmp_P, tmp_L;
                    m_robot->calcTotalMomentum(tmp_P, tmp_L);//calcTotalMomentumは漸化的にWorld周りの並進＋回転運動量を出す
                    cur_momentum_around_COM = tmp_L - m_robot->calcCM().cross(tmp_P);
//                    m_robot->calcTotalMomentumFromJacobian(tmp_P, cur_momentum_around_COM);//calcTotalMomentumFromJacobianは重心ヤコビアンと重心周り角運動量ヤコビアンを用いて重心周りの並進＋回転運動量を出す
                    for(int i=0; i<3; i++){
                        cur_momentum_around_COM_filtered(i) = am_filters[i].passFilter(cur_momentum_around_COM(i));
                    }
                    loop++;
                    if(checkIKConvergence(_ikc_list)){ break; }
                }
                return loop;
            #else
                std::cerr<<"solveFullbodyIKLoop() needs OPENHRP_PACKAGE_VERSION_320 !!!"<<std::endl; return -1;
            #endif
        }
        void solveFullbodyIKOnce(const std::vector<IKConstraint>& _ikc_list){
            #ifdef OPENHRP_PACKAGE_VERSION_320
                // count all valid constraint DOF and allocate Jacobian and vectors
                int C_DOF = 0; // constraint DOF
                for (int i=0;i<_ikc_list.size();i++){ C_DOF += (_ikc_list[i].constraint_weight.array()>0.0).count(); }
                C_DOF += (q_ref_constraint_weight.array()>0.0).count();
                J_all = hrp::dmatrix::Zero(C_DOF, ALL_DOF);
                err_all = hrp::dvector::Zero(C_DOF);
                constraint_weight_all = hrp::dvector::Ones(C_DOF);

                // set end effector constraint into Jacobian
                int cur_cid_all = 0; //current constraint index of all constraints
                for ( int i=0; i<_ikc_list.size(); i++ ) {
                    hrp::Link* link_tgt_ptr = m_robot->link(_ikc_list[i].target_link_name);
                    hrp::dmatrix J_part = hrp::dmatrix::Zero(WS_DOF, ALL_DOF); //全身to拘束点へのヤコビアン
                    hrp::dvector6 dp_part; //pos + rot 速度ではなく差分
                    if(link_tgt_ptr){//ベースリンク，通常リンク共通
                        hrp::Vector3 tgt_cur_pos = link_tgt_ptr->p + link_tgt_ptr->R * _ikc_list[i].localPos;
                        hrp::Matrix33 tgt_cur_rot = link_tgt_ptr->R * _ikc_list[i].localR;
                        dp_part.head(3) =  _ikc_list[i].targetPos - tgt_cur_pos;
                        dp_part.tail(3) = tgt_cur_rot * hrp::omegaFromRotEx(tgt_cur_rot.transpose() * hrp::rotFromRpy(_ikc_list[i].targetRpy));
                        hrp::JointPath tgt_jpath(m_robot->rootLink(), link_tgt_ptr);
                        hrp::dmatrix J_jpath;
                        tgt_jpath.calcJacobian(J_jpath, _ikc_list[i].localPos);
                        for(int id_in_jpath=0; id_in_jpath<tgt_jpath.numJoints(); id_in_jpath++){ J_part.col(tgt_jpath.joint(id_in_jpath)->jointId) = J_jpath.col(id_in_jpath); } //ジョイントパスのJacobianを全身用に並び替え
                        J_part.rightCols(BASE_DOF) = hrp::dmatrix::Identity( WS_DOF,  BASE_DOF );
                        J_part.rightCols(BASE_DOF).topRightCorner(3,3) = hrp::hat(link_tgt_ptr->p - m_robot->rootLink()->p);
                    }else if(!link_tgt_ptr && _ikc_list[i].target_link_name == "COM"){ //重心限定
                        dp_part.head(3) = _ikc_list[i].targetPos - m_robot->calcCM();
//                        dp_part.tail(3) = (_ikc_list[i].targetRpy - cur_momentum_around_COM) * m_dt;// COMのrotはAngulerMomentumとして扱う&差分なのでdtかける
                        dp_part.tail(3) = (_ikc_list[i].targetRpy - cur_momentum_around_COM_filtered) * m_dt;// COMのrotはAngulerMomentumとして扱う&差分なのでdtかける(+フィルタ)
                        m_robot->calcCMJacobian(NULL, J_com);//デフォで右端に3x6のbase->COMのヤコビアンが付いてくる
                        m_robot->calcAngularMomentumJacobian(NULL, J_am);//world座標系での角運動量を生成するヤコビアン(なので本当はCOM周りには使えない)
                        J_part << J_com, J_am;

                    }else{ std::cerr<<"Unknown Link Target !!"<<std::endl; continue; } //不明なリンク指定
                    for(int cid=0; cid<WS_DOF; cid++){
                        if(_ikc_list[i].constraint_weight(cid) > 0.0){
                            J_all.row(cur_cid_all) = J_part.row(cid);

                            LIMIT_MINMAX(dp_part(cid), -0.1,0.1);

                            err_all(cur_cid_all) = dp_part(cid);
                            constraint_weight_all(cur_cid_all) = _ikc_list[i].constraint_weight(cid);
                            cur_cid_all++;
                        }
                    }
                }

                // set desired joint angle constraint into Jacobian
                for(int qid=0; qid<ALL_DOF; qid++){
                    if(q_ref_constraint_weight(qid) > 0.0){
                        J_all.row(cur_cid_all)(qid) = 1;
                        err_all(cur_cid_all) = (qid < J_DOF) ? (q_ref(qid) - m_robot->joint(qid)->q) : (q_ref(qid) - hrp::rpyFromRot(m_robot->rootLink()->R)(qid-J_DOF));
                        LIMIT_MINMAX(err_all(cur_cid_all), -q_ref_max_dq(qid), q_ref_max_dq(qid));
                        constraint_weight_all(cur_cid_all) = q_ref_constraint_weight(qid);
                        cur_cid_all++;
                    }
                }

                // joint limit avoidance (copy from JointPathEx)
                hrp::dvector dq_weight_all_jlim = hrp::dvector::Ones(ALL_DOF);
                for ( int j = 0; j < ALL_DOF ; j++ ) {
                    double jang = (j<J_DOF) ? m_robot->joint(j)->q       :   hrp::rpyFromRot(m_robot->rootLink()->R)(j - J_DOF);
                    double jmax = (j<J_DOF) ? m_robot->joint(j)->ulimit  :   rootlink_rpy_ulimit(j - J_DOF);
                    double jmin = (j<J_DOF) ? m_robot->joint(j)->llimit  :   rootlink_rpy_llimit(j - J_DOF);
                    double e = deg2rad(1);
                    if ( eps_eq(jang, jmax,e) && eps_eq(jang, jmin,e) ) {
                    } else if ( eps_eq(jang, jmax,e) ) {
                        jang = jmax - e;
                    } else if ( eps_eq(jang, jmin,e) ) {
                        jang = jmin + e;
                    }
                    double jlim_avoid_weight;
                    if ( eps_eq(jang, jmax,e) && eps_eq(jang, jmin,e) ) {
                        jlim_avoid_weight = DBL_MAX;
                    } else {
                        jlim_avoid_weight = fabs( (pow((jmax - jmin),2) * (( 2 * jang) - jmax - jmin)) / (4 * pow((jmax - jang),2) * pow((jang - jmin),2)) );
                        if (isnan(jlim_avoid_weight)) jlim_avoid_weight = 0;
                    }
                    if (( jlim_avoid_weight - jlim_avoid_weight_old(j) ) >= 0 ) { // add weight only if q approaching to the limit
                        dq_weight_all_jlim(j) += jlim_avoid_weight;
                    }
                    jlim_avoid_weight_old(j) = jlim_avoid_weight;
                }
                hrp::dvector dq_weight_all_final = dq_weight_all.array() * dq_weight_all_jlim.array();

                // Solvability-unconcerned Inverse Kinematics by Levenberg-Marquardt Method [sugihara:RSJ2009]
                const double wn_const = 1e-6;
                hrp::dmatrix Wn = (static_cast<double>(err_all.transpose() * constraint_weight_all.asDiagonal() * err_all) + wn_const) * hrp::dmatrix::Identity(ALL_DOF, ALL_DOF);
                Wn = dq_weight_all_final.asDiagonal() * Wn;
                hrp::dmatrix H = J_all.transpose() * constraint_weight_all.asDiagonal() * J_all + Wn;
                hrp::dvector g = J_all.transpose() * constraint_weight_all.asDiagonal() * err_all;
                dq_all = H.ldlt().solve(g); // dq_all = H.inverse() * g; is slow

                // rtconf localhost:15005/wbms.rtc set debugLevel 1 とかにしたい
                // debug print
                static int count;
                if(count++ % 10000 == 0){
//                if(true){
                    std::cout<<std::setprecision(2) << "J=\n"<<J_all<<std::setprecision(6)<<std::endl;
                    std::cout<<std::setprecision(2) << "J_all_inv=\n"<<J_all_inv<<std::setprecision(6)<<std::endl;
//                    dbgn(selection_mat);
                    dbgn(H);
                    dbgn(Wn);
                    dbg(g.transpose());
                    dbg(err_all.transpose());
                    dbg(dq_all.transpose());
                    dbg(constraint_weight_all.transpose());
//                    dbg(dq_weight_all_inv.transpose());
                    std::cout<<"q_ans_all\n";
                    for(int i=0;i<m_robot->numJoints();i++)std::cerr<<m_robot->joint(i)->q<<" ";

                    for(int i=0;i<_ikc_list.size();i++){
                        dbg(_ikc_list[i].target_link_name);
                        dbg(_ikc_list[i].targetPos.transpose());
                        dbg(_ikc_list[i].targetRpy.transpose());
                    }
                    std::cout<<std::endl;
                }

                // update joint angles
                for(int i=0;i<dq_all.rows();i++){ if( isnan(dq_all(i)) || isinf(dq_all(i)) ){ std::cerr <<"[FullbodyIK] ERROR nan/inf is found" << std::endl; return;} }
                for(int i=0;i<J_DOF;i++){
                    m_robot->joint(i)->q += dq_all(i);
                    LIMIT_MINMAX(m_robot->joint(i)->q, m_robot->joint(i)->llimit, m_robot->joint(i)->ulimit);
                }

                // update rootlink pos rot
                m_robot->rootLink()->p += dq_all.tail(6).head(3);
                for(int i=0;i<3;i++){
                    if(hrp::rpyFromRot(m_robot->rootLink()->R)(i) < rootlink_rpy_llimit(i) && dq_all.tail(6).tail(3)(i) < 0) dq_all.tail(6).tail(3)(i) = 0;
                    if(hrp::rpyFromRot(m_robot->rootLink()->R)(i) > rootlink_rpy_ulimit(i) && dq_all.tail(6).tail(3)(i) > 0) dq_all.tail(6).tail(3)(i) = 0;
                }

                hrp::Matrix33 dR;
                hrp::Vector3 omega = dq_all.tail(6).tail(3);
                hrp::calcRodrigues(dR, omega.normalized(), omega.norm());
                hrp::Matrix33 R_base_ans = m_robot->rootLink()->R * dR;
                if(!R_base_ans.isUnitary()){
                    std::cerr <<"[FullbodyIK] WARN R_base_ans is not Unitary, normalize via Quaternion" << std::endl;
                    Eigen::Quaternion<double> quat(R_base_ans);
                    quat.normalize();
                    R_base_ans = quat.toRotationMatrix();
                    return;
                }else{
                    m_robot->rootLink()->R = R_base_ans;
                }
                m_robot->calcForwardKinematics();
            #else
                std::cerr<<"solveFullbodyIKOnce() needs OPENHRP_PACKAGE_VERSION_320 !!!"<<std::endl;
            #endif
        }
};

#endif //  FULLBODYIK_H
