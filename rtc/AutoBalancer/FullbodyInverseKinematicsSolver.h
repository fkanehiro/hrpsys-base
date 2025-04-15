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
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        std::string     target_link_name;
        hrp::Vector3    targetPos;
        hrp::Vector3    targetRpy;
        hrp::Vector3    localPos;
        hrp::Matrix33   localR;
        hrp::dvector6   constraint_weight;
        double          pos_precision, rot_precision;
        IKConstraint ()
        :targetPos(         hrp::Vector3::Zero()),
         targetRpy(         hrp::Vector3::Zero()),
         localPos(          hrp::Vector3::Zero()),
         localR(            hrp::Matrix33::Identity()),
         constraint_weight( hrp::dvector6::Ones()),
         pos_precision(1e-4), rot_precision(deg2rad(0.1)){}
        hrp::Vector3    getCurrentTargetPos(const hrp::BodyPtr _robot){ return _robot->link(target_link_name)->p + _robot->link(target_link_name)->R * localPos; }
        hrp::Matrix33   getCurrentTargetRot(const hrp::BodyPtr _robot){ return _robot->link(target_link_name)->R * localR; }
        hrp::Pose3      getCurrentTargetPose(const hrp::BodyPtr _robot){ return hrp::Pose3(getCurrentTargetPos(_robot), getCurrentTargetRot(_robot)); }
};


class FullbodyInverseKinematicsSolver{
    protected:
        const int WS_DOF, BASE_DOF, J_DOF, ALL_DOF;
        const double m_dt;
        hrp::dmatrix J_all;
        hrp::dvector dq_all, err_all, constraint_weight_all, jlim_avoid_weight_old;
        //        BiquadIIRFilterVec am_filters;
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
            dq_weight_all                       = hrp::dvector::Ones(ALL_DOF);
            q_ref                               = hrp::dvector::Zero(ALL_DOF);
            q_ref_constraint_weight             = hrp::dvector::Zero(ALL_DOF);
            q_ref_max_dq                        = hrp::dvector::Constant(ALL_DOF, 1e-2);
            jlim_avoid_weight_old               = hrp::dvector::Zero(ALL_DOF);
            cur_momentum_around_COM             = hrp::Vector3::Zero();
            cur_momentum_around_COM_filtered    = hrp::Vector3::Zero();
            rootlink_rpy_llimit                 = hrp::Vector3::Constant(-DBL_MAX);
            rootlink_rpy_ulimit                 = hrp::Vector3::Constant(DBL_MAX);
//            am_filters.resize(3);
//            am_filters.setParameter(10, 1.0/m_dt, Q_BUTTERWORTH);
//            am_filters.reset(0);
        };
        ~FullbodyInverseKinematicsSolver () {};
        int numStates(){ return ALL_DOF; };
        bool checkIKConvergence(const std::vector<IKConstraint>& _ikc_list){
            for ( int i=0; i<_ikc_list.size(); i++ ) {
                const hrp::Link* link_tgt_ptr = m_robot->link(_ikc_list[i].target_link_name);
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
                const hrp::Vector3    base_p_old = m_robot->rootLink()->p;
                const hrp::Matrix33   base_R_old = m_robot->rootLink()->R;
                const hrp::dvector    q_old = hrp::getQAll(m_robot);
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
//                    cur_momentum_around_COM_filtered = am_filters.passFilter(cur_momentum_around_COM);//フィルター位相遅れのせいか常にウネウネしてる
                    cur_momentum_around_COM_filtered = cur_momentum_around_COM;
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

            // count up all valid constraint DOF and allocate Jacobian and vectors
            const hrp::dmatrix q_select_mat = hrp::to_SelectionMat(dq_weight_all);
            const int VALID_Q_NUM = q_select_mat.rows();
            if(VALID_Q_NUM == 0){ return; }
            int VALID_C_NUM = 0; // count up valid constraint num
            for (int i=0;i<_ikc_list.size();i++){ VALID_C_NUM += (_ikc_list[i].constraint_weight.array()>0.0).count(); }// count up valid end effector constraint num
            VALID_C_NUM += (q_ref_constraint_weight.array()>0.0).count();// count up valid reference joint angle constraint num
            J_all = hrp::dmatrix::Zero(VALID_C_NUM, VALID_Q_NUM);
            err_all = hrp::dvector::Zero(VALID_C_NUM);
            constraint_weight_all = hrp::dvector::Ones(VALID_C_NUM);

            // set end effector constraints into Jacobian
            int CURRENT_C_COUNT = 0;
            for ( int i=0; i<_ikc_list.size(); i++ ) {
                hrp::Link* link_tgt_ptr = m_robot->link(_ikc_list[i].target_link_name);
                hrp::dmatrix J_part = hrp::dmatrix::Zero(WS_DOF, ALL_DOF); //全身to拘束点へのヤコビアン
                hrp::dvector6 dp_part; //pos + rot 速度ではなく差分
                if(link_tgt_ptr){//ベースリンク，通常リンク共通
                    hrp::Vector3 tgt_cur_pos = link_tgt_ptr->p + link_tgt_ptr->R * _ikc_list[i].localPos;
                    hrp::Matrix33 tgt_cur_rot = link_tgt_ptr->R * _ikc_list[i].localR;
                    dp_part.head(3) = _ikc_list[i].targetPos - tgt_cur_pos;
                    dp_part.tail(3) = tgt_cur_rot * hrp::omegaFromRotEx(tgt_cur_rot.transpose() * hrp::rotFromRpy(_ikc_list[i].targetRpy));
                    hrp::JointPath tgt_jpath(m_robot->rootLink(), link_tgt_ptr);
                    hrp::dmatrix J_jpath;
                    tgt_jpath.calcJacobian(J_jpath, _ikc_list[i].localPos);
                    for(int id_in_jpath=0; id_in_jpath<tgt_jpath.numJoints(); id_in_jpath++){ J_part.col(tgt_jpath.joint(id_in_jpath)->jointId) = J_jpath.col(id_in_jpath); } //ジョイントパスのJacobianを全身用に並び替え
                    J_part.rightCols(BASE_DOF) = hrp::dmatrix::Identity( WS_DOF,  BASE_DOF );
                    J_part.rightCols(BASE_DOF).topRightCorner(3,3) = - hrp::hat(tgt_cur_pos - m_robot->rootLink()->p);
                }
                else if(!link_tgt_ptr && _ikc_list[i].target_link_name == "COM"){ //重心限定
                    dp_part.head(3) = _ikc_list[i].targetPos - m_robot->calcCM();
                    dp_part.tail(3) = (_ikc_list[i].targetRpy - cur_momentum_around_COM_filtered) * m_dt;// COMのrotはAngulerMomentumとして扱う&差分なのでdtかける(+フィルタ)
                    hrp::dmatrix J_com, J_am;
                    m_robot->calcCMJacobian(NULL, J_com);//デフォで右端に3x6のbase->COMのヤコビアンが付いてくる
                    m_robot->calcAngularMomentumJacobian(NULL, J_am);//base=NULLの時は重心周りの角運動量っぽい？
                    J_part << J_com, J_am;
                }
                else{
                    std::cerr<<"Unknown Link Target !! "<< _ikc_list[i].target_link_name << std::endl; continue;  //不明なリンク指定
                }
                // set one of the end effector constraints into Jacobian
                hrp::dmatrix c_part_selection_mat = hrp::to_SelectionMat(_ikc_list[i].constraint_weight);   // select only valid end effector constraint
                if(c_part_selection_mat.rows() != 0 && c_part_selection_mat.cols() != 0 ){

                    // 明らかに可到達でない条件の誤差は頭打ちしないと，いくら重み付きでも引きずられる?
                    dp_part = dp_part.cwiseMin(hrp::dvector6::Constant(0.1)).cwiseMax(hrp::dvector6::Constant(-0.1));

                    J_all.middleRows                (CURRENT_C_COUNT, c_part_selection_mat.rows()) = c_part_selection_mat * J_part * q_select_mat.transpose();
                    err_all.segment                 (CURRENT_C_COUNT, c_part_selection_mat.rows()) = c_part_selection_mat * dp_part;
                    constraint_weight_all.segment   (CURRENT_C_COUNT, c_part_selection_mat.rows()) = c_part_selection_mat * _ikc_list[i].constraint_weight;
                    CURRENT_C_COUNT += c_part_selection_mat.rows();
                }
            }

            // set reference joint angle constraints into Jacobian
            const hrp::dmatrix q_ref_selection_mat = hrp::to_SelectionMat(q_ref_constraint_weight);
            if(q_ref_selection_mat.rows() != 0 && q_ref_selection_mat.cols() != 0 ){
                J_all.middleRows                (CURRENT_C_COUNT, q_ref_selection_mat.rows()) = q_ref_selection_mat * hrp::dmatrix::Identity(ALL_DOF,ALL_DOF) * q_select_mat.transpose();
                err_all.segment                 (CURRENT_C_COUNT, q_ref_selection_mat.rows()) = q_ref_selection_mat * ((q_ref - hrp::getRobotStateVec(m_robot)).cwiseMin(q_ref_max_dq).cwiseMax(-q_ref_max_dq));
                constraint_weight_all.segment   (CURRENT_C_COUNT, q_ref_selection_mat.rows()) = q_ref_selection_mat * q_ref_constraint_weight;
                CURRENT_C_COUNT += q_ref_selection_mat.rows();
            }
            if(CURRENT_C_COUNT != VALID_C_NUM){ std::cerr <<"[FullbodyIK] CURRENT_C_COUNT != VALID_C_NUM, something wrong !" << std::endl; }

            // joint limit avoidance (copy from JointPathEx)
            hrp::dvector dq_weight_all_jlim = hrp::dvector::Ones(ALL_DOF);
            for ( int j = 0; j < ALL_DOF ; j++ ) {
                double jang, jmax, jmin;
                if(j<J_DOF){
                    jang = m_robot->joint(j)->q;
                    jmax = m_robot->joint(j)->ulimit;
                    jmin = m_robot->joint(j)->llimit;
                }else if(j<J_DOF+3){
                    continue;// do nothing about base link pos
                }else{
                    const int rpy_id = j - (J_DOF+3);
                    if(!(0 <= rpy_id && rpy_id < 3)){ std::cerr <<"[FullbodyIK] !(0 <= rpy_id && rpy_id < 3), something wrong !" << std::endl; }
                    jang = hrp::rpyFromRot(m_robot->rootLink()->R)(rpy_id);
                    jmax = rootlink_rpy_ulimit(rpy_id);
                    jmin = rootlink_rpy_llimit(rpy_id);
                }
                const double e = deg2rad(1);
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
                    if (std::isnan(jlim_avoid_weight)) jlim_avoid_weight = 0;
                }
                if (( jlim_avoid_weight - jlim_avoid_weight_old(j) ) >= 0 ) { // add weight only if q approaching to the limit
                    dq_weight_all_jlim(j) += jlim_avoid_weight;
                }
                jlim_avoid_weight_old(j) = jlim_avoid_weight;
            }
            hrp::dvector dq_weight_all_final = dq_weight_all.array() * dq_weight_all_jlim.array();
            dq_weight_all_final = q_select_mat * dq_weight_all_final;

            // Solvability-unconcerned Inverse Kinematics by Levenberg-Marquardt Method [sugihara:RSJ2009]
            // q = q + H^(-1) * g
            // g = J^T * We * e
            // H = J^T * We * J + Wn
            // Wn = (e^T * We * e) * 1 + \bar{Wn}   // \bar{Wn} = \bar{wn} * E
            //    = (e^T * We * e + \bar{wn}) * E
            // Wn = Wn * Wq                         // modify to insert dq weight
            const double wn_const = 1e-6;
            const hrp::dmatrix Wn = (static_cast<double>(err_all.transpose() * constraint_weight_all.asDiagonal() * err_all) + wn_const) * dq_weight_all_final.asDiagonal();
            const hrp::dmatrix H = J_all.transpose() * constraint_weight_all.asDiagonal() * J_all + Wn;
            const hrp::dvector g = J_all.transpose() * constraint_weight_all.asDiagonal() * err_all;
            dq_all = H.ldlt().solve(g); // dq_all = H.inverse() * g; is slow

            // rtconf localhost:15005/wbms.rtc set debugLevel 1 とかにしたい
            static int count;
#if 0
            if(count++ % 10000 == 0){
                std::cout<<std::setprecision(2) << "J=\n"<<J_all<<std::setprecision(6)<<std::endl;
                dbg(J_all.rows());
                dbg(J_all.cols());
                dbgn(H);
                dbg(H.rows());
                dbg(H.cols());
                dbgn(Wn);
                dbg(Wn.rows());
                dbg(Wn.cols());
                dbgv(g);
                dbg(g.rows());
                dbg(g.cols());
                dbgv(err_all);
                dbg(err_all.rows());
                dbg(err_all.cols());
                dbgv(dq_all);
                dbg(dq_all.rows());
                dbg(dq_all.cols());
                dbgv(constraint_weight_all);
                dbg(constraint_weight_all.rows());
                dbg(constraint_weight_all.cols());
                dbgn(q_ref_selection_mat);
                dbgv(q_ref);
                dbgv(hrp::getRobotStateVec(m_robot));
                std::cout<<std::endl;
            }
#endif
            // update joint angles
            for(int i=0;i<dq_all.rows();i++){ if( std::isnan(dq_all(i)) || std::isinf(dq_all(i)) ){ std::cerr <<"[FullbodyIK] ERROR nan/inf is found" << std::endl; return;} }
            dq_all = q_select_mat.transpose() * dq_all;
            for(int i=0;i<J_DOF;i++){
                m_robot->joint(i)->q += dq_all(i);
                LIMIT_MINMAX(m_robot->joint(i)->q, m_robot->joint(i)->llimit, m_robot->joint(i)->ulimit);
            }

            // rootlink rpy limit ???
            for(int i=0;i<3;i++){
                if(hrp::rpyFromRot(m_robot->rootLink()->R)(i) < rootlink_rpy_llimit(i) && dq_all.tail(6).tail(3)(i) < 0) dq_all.tail(6).tail(3)(i) = 0;
                if(hrp::rpyFromRot(m_robot->rootLink()->R)(i) > rootlink_rpy_ulimit(i) && dq_all.tail(6).tail(3)(i) > 0) dq_all.tail(6).tail(3)(i) = 0;
            }

            // update rootlink pos rot
            m_robot->rootLink()->p += dq_all.tail(6).head(3);
            const hrp::Vector3 omega = dq_all.tail(6).tail(3);
            hrp::Matrix33 dR;
            if(omega.norm() > 1e-12){
                hrp::calcRodrigues(dR, omega.normalized(), omega.norm());
            }else{
                dR = hrp::Matrix33::Identity();
            }
            rats::rotm3times(m_robot->rootLink()->R, dR, m_robot->rootLink()->R); // safe rot operation with quartanion normalization
            if(!m_robot->rootLink()->R.isUnitary()){
                std::cerr <<"[FullbodyIK] WARN m_robot->rootLink()->R is not Unitary, something wrong !" << std::endl;
            }
            m_robot->calcForwardKinematics();
            #else
                std::cerr<<"solveFullbodyIKOnce() needs OPENHRP_PACKAGE_VERSION_320 !!!"<<std::endl;
            #endif
        }
};

#endif //  FULLBODYIK_H
