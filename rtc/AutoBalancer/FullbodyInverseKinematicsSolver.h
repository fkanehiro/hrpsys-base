#ifndef FULLBODYIK_H
#define FULLBODYIK_H

#include <float.h>
#include <hrpModel/Body.h>
#include <hrpUtil/MatrixSolvers.h>
#include "../ImpedanceController/JointPathEx.h"
#include <unsupported/Eigen/MatrixFunctions>
#include "SimpleFullbodyInverseKinematicsSolver.h"
#include "../TorqueFilter/IIRFilter.h"

//tips
#define LIMIT_MINMAX(x,min,max) ((x= (x<min  ? min : x<max ? x : max)))
#define deg2rad(x) ((x)*M_PI/180)
#define rad2deg(rad) (rad * 180 / M_PI)
#define eps_eq(a, b, c)  (fabs((a)-(b)) <= c)
#define dbg(var) std::cout<<#var"= "<<(var)<<std::endl
#define dbgn(var) std::cout<<#var"= "<<std::endl<<(var)<<std::endl
namespace hrp{
    static hrp::dvector getQAll(const hrp::BodyPtr _robot){ hrp::dvector tmp(_robot->numJoints()); for(int i=0;i<_robot->numJoints();i++){ tmp(i) = _robot->joint(i)->q; } return tmp; };
    static void setQAll(hrp::BodyPtr _robot, const hrp::dvector in){ assert(in.size() <= _robot->numJoints()); for(int i=0;i<_robot->numJoints();i++){ _robot->joint(i)->q = in(i); } };
}


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
};


class FullbodyInverseKinematicsSolver : public SimpleFullbodyInverseKinematicsSolver{
    protected:
        hrp::dmatrix J_all, J_all_inv, J_com, J_am;
        hrp::dvector dq_all, err_all, constraint_weight_all, avoid_weight_gain;
        const int WS_DOF, BASE_DOF, J_DOF, ALL_DOF;
        std::vector<IIRFilter> am_filters;
    public:
        hrp::dvector dq_weight_all;
        hrp::dvector q_ref, q_ref_pullback_gain;
        hrp::Vector3 cur_momentum_around_COM, cur_momentum_around_COM_filtered;
        FullbodyInverseKinematicsSolver (hrp::BodyPtr _robot, const std::string& _print_str, const double _dt)
        : SimpleFullbodyInverseKinematicsSolver(_robot,_print_str, _dt),
          WS_DOF(6),
          BASE_DOF(6),
          J_DOF(_robot->numJoints()),
          ALL_DOF(J_DOF+BASE_DOF) {
            dq_weight_all = hrp::dvector::Ones(ALL_DOF);
            dq_all = q_ref = q_ref_pullback_gain = hrp::dvector::Zero(ALL_DOF);
            cur_momentum_around_COM = cur_momentum_around_COM_filtered = hrp::Vector3::Zero();
            avoid_weight_gain = hrp::dvector::Constant(ALL_DOF, 1e12);
            am_filters.resize(3);
            for(int i=0;i<3;i++){
              am_filters[i].setParameterAsBiquad(10, 1/std::sqrt(2), 1.0/m_dt);
              am_filters[i].reset();
            }
        };
        ~FullbodyInverseKinematicsSolver () {};
        bool checkIKConvergence(const hrp::BodyPtr _robot, const std::vector<IKConstraint>& _ikc_list){
            for ( int i=0; i<_ikc_list.size(); i++ ) {
                hrp::Link* link_tgt_ptr = _robot->link(_ikc_list[i].target_link_name);
                hrp::Vector3 pos_err, rot_err;
                if(link_tgt_ptr){
                    pos_err = _ikc_list[i].targetPos - (link_tgt_ptr->p + link_tgt_ptr->R * _ikc_list[i].localPos);
                    rats::difference_rotation(rot_err, (link_tgt_ptr->R * _ikc_list[i].localR), hrp::rotFromRpy(_ikc_list[i].targetRpy));
                }
                else if(!link_tgt_ptr && _ikc_list[i].target_link_name == "COM"){  // COM
                    pos_err = _ikc_list[i].targetPos - (_robot->calcCM() + _ikc_list[i].localR * _ikc_list[i].localPos);
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
        int solveFullbodyIKLoop (hrp::BodyPtr _robot, const std::vector<IKConstraint>& _ikc_list, const int _max_iteration) {
//#define OPENHRP_PACKAGE_VERSION_320
            #ifdef OPENHRP_PACKAGE_VERSION_320
                hrp::Vector3 base_p_old = _robot->rootLink()->p;
                hrp::Matrix33 base_R_old = _robot->rootLink()->R;
                hrp::dvector q_old(J_DOF);
                for(int i=0;i<J_DOF;i++){ q_old(i) = _robot->joint(i)->q; }
                unsigned int loop;
                for(loop=0; loop < _max_iteration;){
                    solveFullbodyIKOnce(_robot, _ikc_list);
                    //check ang moment
                    _robot->rootLink()->v = (_robot->rootLink()->p - base_p_old)/ m_dt;
                    _robot->rootLink()->w = base_R_old * omegaFromRotEx(base_R_old.transpose() * _robot->rootLink()->R) / m_dt;
                    for(int i=0;i<J_DOF;i++){ _robot->joint(i)->dq = (_robot->joint(i)->q - q_old(i)) / m_dt; }
                    _robot->calcForwardKinematics(true,false);
                    hrp::Vector3 tmp_P, tmp_L;
                    _robot->calcTotalMomentum(tmp_P, tmp_L);//calcTotalMomentumは漸化的にWorld周りの並進＋回転運動量を出す
                    cur_momentum_around_COM = tmp_L - _robot->calcCM().cross(tmp_P);
//                    _robot->calcTotalMomentumFromJacobian(tmp_P, cur_momentum_around_COM);//calcTotalMomentumFromJacobianは重心ヤコビアンと重心周り角運動量ヤコビアンを用いて重心周りの並進＋回転運動量を出す
                    for(int i=0; i<3; i++){
                        cur_momentum_around_COM_filtered(i) = am_filters[i].passFilter(cur_momentum_around_COM(i));
                    }
                    loop++;
                    if(checkIKConvergence(_robot, _ikc_list)){ break; }
                }
                return loop;
            #else
                std::cerr<<"solveFullbodyIKLoop() needs OPENHRP_PACKAGE_VERSION_320 !!!"<<std::endl; return -1;
            #endif
        }
        void solveFullbodyIKOnce(hrp::BodyPtr _robot, const std::vector<IKConstraint>& _ikc_list){
            #ifdef OPENHRP_PACKAGE_VERSION_320
                J_all = hrp::dmatrix::Zero(WS_DOF*_ikc_list.size(), ALL_DOF);
                err_all = hrp::dvector::Zero(WS_DOF*_ikc_list.size());
                constraint_weight_all = hrp::dvector::Ones(WS_DOF*_ikc_list.size());
                //リファレンスに微少量戻す
                for(int i=0;i<J_DOF;i++){
                    const double diff = q_ref(i) - _robot->joint(i)->q;
                    _robot->joint(i)->q += diff * q_ref_pullback_gain(i);
                }
                for(int i=0;i<J_DOF;i++){ LIMIT_MINMAX(_robot->joint(i)->q, _robot->joint(i)->llimit, _robot->joint(i)->ulimit); }
                _robot->calcForwardKinematics();
                //ヤコビアンと各ベクトル生成
                for ( int i=0; i<_ikc_list.size(); i++ ) {
                    hrp::Link* link_tgt_ptr = _robot->link(_ikc_list[i].target_link_name);
                    hrp::dmatrix J_part = hrp::dmatrix::Zero(WS_DOF, ALL_DOF); //全身to拘束点へのヤコビアン
                    hrp::Vector3 dp_part, dr_part; //速度ではなく差分
                    //ベースリンク，通常リンク，重心で場合分け
                    if(link_tgt_ptr){//ベースリンク，通常リンク共通
                        hrp::Vector3 tgt_cur_pos = link_tgt_ptr->p + link_tgt_ptr->R * _ikc_list[i].localPos;
                        hrp::Matrix33 tgt_cur_rot = link_tgt_ptr->R * _ikc_list[i].localR;
                        dp_part =  _ikc_list[i].targetPos - tgt_cur_pos;
                        dr_part = tgt_cur_rot * omegaFromRotEx(tgt_cur_rot.transpose() * hrp::rotFromRpy(_ikc_list[i].targetRpy));
                        hrp::JointPathEx tgt_jpath(_robot, _robot->rootLink(), link_tgt_ptr, m_dt, false, "");
                        hrp::dmatrix J_jpath;
                        tgt_jpath.calcJacobian(J_jpath, _ikc_list[i].localPos);
                        for(int id_in_jpath=0; id_in_jpath<tgt_jpath.numJoints(); id_in_jpath++){ //ジョイントパスのJacobianを全身用に並び替え
                            int id_in_body = tgt_jpath.joint(id_in_jpath)->jointId; //全身でのjoint番号
                            J_part.col(id_in_body) = J_jpath.col(id_in_jpath);
                        }
                        J_part.rightCols(BASE_DOF) = hrp::dmatrix::Identity( WS_DOF,  BASE_DOF );
                        J_part.rightCols(BASE_DOF).topRightCorner(3,3) = hrp::hat(link_tgt_ptr->p - _robot->rootLink()->p);
                    }
                    else if(!link_tgt_ptr && _ikc_list[i].target_link_name == "COM"){ //重心限定
                        dp_part = _ikc_list[i].targetPos - _robot->calcCM();
//                        dr_part = (_ikc_list[i].targetRpy - cur_momentum_around_COM) * m_dt;// COMのrotはAngulerMomentumとして扱う&差分なのでdtかける
                        dr_part = (_ikc_list[i].targetRpy - cur_momentum_around_COM_filtered) * m_dt;// COMのrotはAngulerMomentumとして扱う&差分なのでdtかける
                        _robot->calcCMJacobian(NULL, J_com);//デフォで右端に3x6のbase->COMのヤコビアンが付いてくる
                        _robot->calcAngularMomentumJacobian(NULL, J_am);//world座標系での角運動量を生成するヤコビアン(なので本当はCOM周りには使えない)
                        J_part << J_com, J_am;
                    }
                    else{ std::cerr<<"Unknown Link Target !!"<<std::endl; continue; } //不明なリンク指定
                    err_all.segment(WS_DOF*i,WS_DOF) << dp_part, dr_part;
                    constraint_weight_all.segment(WS_DOF*i, WS_DOF) = _ikc_list[i].constraint_weight;
                    J_all.middleRows(WS_DOF*i, WS_DOF) = J_part;
                }
                // delete the row in Jacobian if the constraint weight is 0
                hrp::dmatrix selection_mat = hrp::dmatrix::Zero((constraint_weight_all.array()>0.0).count(), WS_DOF*_ikc_list.size());
                for(int i=0, j=0; i<constraint_weight_all.rows(); i++){ if(constraint_weight_all(i) > 0.0){ selection_mat(j, i) = 1.0; j++; } }
                J_all = selection_mat * J_all;
                err_all = selection_mat * err_all;
                constraint_weight_all = selection_mat * constraint_weight_all;
                // joint limit avoidance (copy from JointPathEx)
                hrp::dvector dq_weight_all_jlim = dq_weight_all;
                for ( int j = 0; j < _robot->numJoints() ; j++ ) {
                    double jang = _robot->joint(j)->q;
                    double jmax = _robot->joint(j)->ulimit;
                    double jmin = _robot->joint(j)->llimit;
                    double e = deg2rad(1);
                    if ( eps_eq(jang, jmax,e) && eps_eq(jang, jmin,e) ) {
                    } else if ( eps_eq(jang, jmax,e) ) {
                        jang = jmax - e;
                    } else if ( eps_eq(jang, jmin,e) ) {
                        jang = jmin + e;
                    }
                    double r;
                    if ( eps_eq(jang, jmax,e) && eps_eq(jang, jmin,e) ) {
                        r = DBL_MAX;
                    } else {
                        r = fabs( (pow((jmax - jmin),2) * (( 2 * jang) - jmax - jmin)) /
                                (4 * pow((jmax - jang),2) * pow((jang - jmin),2)) );
                        if (isnan(r)) r = 0;
                    }
                    if (( r - avoid_weight_gain(j) ) >= 0 ) {
                        dq_weight_all_jlim(j) *= ( 1.0 / ( 1.0 + r) );
                    } else {
                        dq_weight_all_jlim(j) *= 1.0;
                    }
                    avoid_weight_gain(j) = r;
                }
                //Solvability-unconcerned Inverse Kinematics by Levenberg-Marquardt Method [sugihara:RSJ2009]
                const double wn_const = 1e-6;
                hrp::dmatrix Wn = (static_cast<double>(err_all.transpose() * constraint_weight_all.asDiagonal() * err_all) + wn_const) * hrp::dmatrix::Identity(ALL_DOF, ALL_DOF);
                hrp::dvector dq_weight_all_inv = dq_weight_all_jlim.array().inverse();//重み小でdq小にするため
                Wn = dq_weight_all_inv.asDiagonal() * Wn;
                hrp::dmatrix H = J_all.transpose() * constraint_weight_all.asDiagonal() * J_all + Wn;
                hrp::dvector g = J_all.transpose() * constraint_weight_all.asDiagonal() * err_all;
                dq_all = H.inverse() * g;

    //            // debug print
    //            static int count;
    //            if(count++ % 10000 == 0){
    //                std::cout<<std::setprecision(2) << "J=\n"<<J_all<<std::setprecision(6)<<std::endl;
    //                std::cout<<std::setprecision(2) << "J_all_inv=\n"<<J_all_inv<<std::setprecision(6)<<std::endl;
    //                dbgn(selection_mat);
    //                dbgn(H);
    //                dbg(Wn(0,0));
    //                dbg(g.transpose());
    //                dbg(err_all.transpose());
    //                dbg(dq_all.transpose());
    //                dbg(constraint_weight_all.transpose());
    //                dbg(dq_weight_all_inv.transpose());
    //                std::cout<<"q_ans_all\n";
    //                for(int i=0;i<_robot->numJoints();i++)std::cerr<<_robot->joint(i)->q<<" ";
    //                std::cout<<std::endl;
    //            }

                // detect numerical error in base rot matrix or nan/inf in joint angles
                hrp::Matrix33 eWx = hrp::hat(dq_all.segment(J_DOF+3,3)).exp();
                hrp::Matrix33 R_base_ans = eWx * _robot->rootLink()->R;
                // normalize rot mat via Quaternion
                Eigen::Quaternion<double> quat(R_base_ans);
                quat.normalize();
                R_base_ans = quat.toRotationMatrix();
                if(!R_base_ans.isUnitary()){ std::cerr <<"ERROR R_base_ans is not Unitary" << std::endl; return; }
                for(int i=0;i<dq_all.rows();i++){ if( isnan(dq_all(i)) || isinf(dq_all(i)) ){ std::cerr <<"ERROR nan/inf is found" << std::endl; return;} }
                //関節角+ベース位置姿勢更新
                for(int i=0;i<J_DOF;i++){
                    _robot->joint(i)->q += dq_all(i);
                    LIMIT_MINMAX(_robot->joint(i)->q, _robot->joint(i)->llimit, _robot->joint(i)->ulimit);
                }
                _robot->rootLink()->p += dq_all.segment(J_DOF,3);
//                _robot->rootLink()->R = R_base_ans;

                for(int i=0;i<2;i++){
                    if(hrp::rpyFromRot(_robot->rootLink()->R)(i) < deg2rad(-10) && dq_all.tail(6).tail(3)(i) < 0){
                        dq_all.tail(6).tail(3)(i) = 0;
                    }
                    if(hrp::rpyFromRot(_robot->rootLink()->R)(i) > deg2rad(10) && dq_all.tail(6).tail(3)(i) > 0){
                        dq_all.tail(6).tail(3)(i) = 0;
                    }
                }
                _robot->rootLink()->R = hrp::rotFromRpy((hrp::rpyFromRot(_robot->rootLink()->R) + dq_all.tail(6).tail(3)));


                _robot->calcForwardKinematics();
            #else
                std::cerr<<"solveFullbodyIKOnce() needs OPENHRP_PACKAGE_VERSION_320 !!!"<<std::endl;
            #endif
        }
        void revertRobotStateToCurrentAll (){
            hrp::setQAll(m_robot, qorg);
            m_robot->rootLink()->p = current_root_p;
            m_robot->rootLink()->R = current_root_R;
            m_robot->calcForwardKinematics();
        };

    protected:
        hrp::Vector3 omegaFromRotEx(const hrp::Matrix33& r) {//copy from JointPathEx.cpp
            using ::std::numeric_limits;
            double alpha = (r(0,0) + r(1,1) + r(2,2) - 1.0) / 2.0;
            if(fabs(alpha - 1.0) < 1.0e-12) {   //th=0,2PI;
                return hrp::Vector3::Zero();
            } else {
                double th = acos(alpha);
                double s = sin(th);
                if (s < numeric_limits<double>::epsilon()) {   //th=PI
                    return hrp::Vector3( sqrt((r(0,0)+1)*0.5)*th, sqrt((r(1,1)+1)*0.5)*th, sqrt((r(2,2)+1)*0.5)*th );
                }
                double k = -0.5 * th / s;
                return hrp::Vector3( (r(1,2) - r(2,1)) * k, (r(2,0) - r(0,2)) * k, (r(0,1) - r(1,0)) * k );
            }
        }
};


// // multi-thread IK (answer will delay for 1 control loop)
//class FullbodyInverseKinematicsSolverMT : public FullbodyInverseKinematicsSolver{
//    protected:
//        hrp::BodyPtr _robot_copy;
//        pthread_t ik_thread;
//        pthread_mutex_t ik_body_mutex;  // Mutex
//        int cur_ik_loop;
//        std::vector<IKConstraint> ikc_list;
//        bool is_first;
//        bool ik_thread_kill;
//        bool ik_thread_ik_required;
//        hrp::Vector3 base_p_old;
//        hrp::Matrix33 base_R_old;
//        hrp::dvector q_old;
//    public:
//        FullbodyInverseKinematicsSolverMT (hrp::BodyPtr& _robot, const std::string& _print_str, const double _dt) : FullbodyInverseKinematicsSolver(_robot,_print_str, _dt) {
//            _robot_copy = hrp::BodyPtr(new hrp::Body(*_robot));
//            cur_ik_loop = 0;
//            is_first = true;
//            ik_thread_kill = false;
//            ik_thread_ik_required = false;
//        };
//        ~FullbodyInverseKinematicsSolverMT () {
//            pthread_cancel(ik_thread);
//            ik_thread_kill = true;
//            pthread_join(ik_thread, NULL );
//            std::cerr<<"[FullbodyInverseKinematicsSolverMT] thread killed successfully"<<std::endl;
//        };
//        int solveFullbodyIKLoopMT (hrp::BodyPtr _robot, const std::vector<IKConstraint>& _ikc_list, const int _max_iteration) {
//            if(is_first){
//                for(int i=0;i<J_DOF;i++){
//                    _robot_copy->joint(i)->q = _robot->joint(i)->q;
//                    _robot_copy->joint(i)->ulimit = _robot->joint(i)->ulimit;
//                    _robot_copy->joint(i)->llimit = _robot->joint(i)->llimit;
//                }
//                _robot_copy->rootLink()->p = _robot->rootLink()->p;
//                _robot_copy->rootLink()->R = _robot->rootLink()->R;
//                pthread_mutex_init( &ik_body_mutex, NULL );
//                pthread_create(&ik_thread, NULL, launchThread, this);
//                is_first = false;
//            }
//            for(int i=0;i<J_DOF;i++){
//                _robot->joint(i)->q = _robot_copy->joint(i)->q;
//            }
//            _robot->rootLink()->p = _robot_copy->rootLink()->p;
//            _robot->rootLink()->R = _robot_copy->rootLink()->R;
//            _robot->calcForwardKinematics();
//            int result = cur_ik_loop;
//
//            if(pthread_mutex_lock( &ik_body_mutex ) != 0){std::cerr<<"[FullbodyInverseKinematicsSolverMT] pthread_mutex_lock err "<<std::endl;}
//
//            base_p_old = _robot_copy->rootLink()->p;
//            base_R_old = _robot_copy->rootLink()->R;
//            q_old.resize(J_DOF);
//            for(int i=0;i<J_DOF;i++){ q_old(i) = _robot_copy->joint(i)->q; }
//
//            cur_ik_loop = 0;
//            ikc_list = _ikc_list;
//            ik_thread_ik_required = true;
//            if(pthread_mutex_unlock( &ik_body_mutex ) != 0){std::cerr<<"[FullbodyInverseKinematicsSolverMT] pthread_mutex_unlock err "<<std::endl;}
//            return result;
//        }
//        void ik_loop(){
//            std::cerr<<"[FullbodyInverseKinematicsSolverMT] start thread"<<std::endl;
//            while(1){
//                usleep(0);//これ無いとmutex解放しない？
//                if(pthread_mutex_lock( &ik_body_mutex ) != 0){std::cerr<<"[FullbodyInverseKinematicsSolverMT] pthread_mutex_lock2 err "<<std::endl;}
//                if(ik_thread_ik_required){
//                    solveFullbodyIK(_robot_copy, ikc_list);
//                    //check ang moment
//                    _robot_copy->rootLink()->v = (_robot_copy->rootLink()->p - base_p_old)/ m_dt;
//                    _robot_copy->rootLink()->w = base_R_old * omegaFromRotEx(base_R_old.transpose() * _robot_copy->rootLink()->R) / m_dt;
//                    for(int i=0;i<J_DOF;i++){ _robot_copy->joint(i)->dq = (_robot_copy->joint(i)->q - q_old(i)) / m_dt; }
//                    _robot_copy->calcForwardKinematics(true,false);
////                    _robot_copy->calcTotalMomentum(cur_momentum_P, cur_momentum_L);
//                    cur_ik_loop++;
//                    if(cur_ik_loop > 10000){std::cerr<<"[FullbodyInverseKinematicsSolverMT] thread time out"<<std::endl;break;}//異常終了でスレッドが残ってしまった時にタイムアウト終了
//                }
//                if(pthread_mutex_unlock( &ik_body_mutex ) != 0){std::cerr<<"[FullbodyInverseKinematicsSolverMT] pthread_mutex_unlock2 err "<<std::endl;}
//                if(ik_thread_kill){break;}
//            }
//            std::cerr<<"[FullbodyInverseKinematicsSolverMT] end of thread"<<std::endl;
//        }
//        static void* launchThread(void *pParam) {
//            reinterpret_cast<FullbodyInverseKinematicsSolverMT*>(pParam)->ik_loop();
//            return NULL;
//        }
//};


#endif //  FULLBODYIK_H

