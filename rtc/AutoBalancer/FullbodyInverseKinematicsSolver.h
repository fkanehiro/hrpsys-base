#ifndef FULLBODYIK_H
#define FULLBODYIK_H

#include <float.h>
#include <hrpModel/Body.h>
#include <hrpUtil/MatrixSolvers.h>
#include "../ImpedanceController/JointPathEx.h"
#include "SimpleFullbodyInverseKinematicsSolver.h"
#include <unsupported/Eigen/MatrixFunctions>

#define LIMIT_MINMAX(x,min,max) ((x= (x<min  ? min : x<max ? x : max)))
#define deg2rad(x)((x)*M_PI/180)
#define rad2deg(rad) (rad * 180 / M_PI)
#define eps_eq(a, b, c)  (fabs((a)-(b)) <= c)

class IKConstraintParam {
  public:
    std::string target_link_name;
    hrp::Vector3 targetPos;
    hrp::Matrix33 targetR;
    hrp::Vector3 localPos;
    hrp::Matrix33 localR;
    hrp::dvector6 weight_vec;
    hrp::ivector selection_vec;
    double pos_precision, rot_precision;

    IKConstraintParam ()
      :targetPos(hrp::Vector3::Zero()),
       targetR(hrp::Matrix33::Identity()),
       localPos(hrp::Vector3::Zero()),
       localR(hrp::Matrix33::Identity()),
       weight_vec(hrp::dvector6::Ones()),
       selection_vec(hrp::ivector::Ones(6)),
       pos_precision(1e-3), rot_precision(1e-2)
    {};
};


class FullbodyInverseKinematicsSolver : public SimpleFullbodyInverseKinematicsSolver{
  protected:
    hrp::dmatrix J_all, J_all_inv, J_com, J_am;
    hrp::dvector dq_all, dp_ee_all, weight_vec_all;
    hrp::ivector selection_vec_all;
    int WS_DOF, BASE_DOF, J_DOF, ALL_DOF;
    hrp::Vector3 cur_P,cur_L;

  public:
    hrp::dvector optional_weight_vector;
    hrp::dvector reference_q, reference_gain;
    FullbodyInverseKinematicsSolver (hrp::BodyPtr& _robot, const std::string& _print_str, const double _dt) : SimpleFullbodyInverseKinematicsSolver(_robot,_print_str, _dt) {
      WS_DOF = 6;
      BASE_DOF = 6;
      J_DOF = m_robot->numJoints();
      ALL_DOF = BASE_DOF + J_DOF;
      optional_weight_vector = hrp::dvector::Ones(ALL_DOF);
      reference_q = reference_gain = hrp::dvector::Zero(ALL_DOF);
      dq_all = hrp::dvector::Zero( ALL_DOF );
    };
    ~FullbodyInverseKinematicsSolver () {};
    // Solve fullbody IK using limb IK
    int solveFullbodyIKLoop (const std::vector<IKConstraintParam>& _ik_tgt_list, const int _max_iteration) {
      hrp::Vector3 base_p_old = m_robot->rootLink()->p;
      hrp::Matrix33 base_R_old = m_robot->rootLink()->R;
      hrp::dvector q_old(J_DOF);
      for(int i=0;i<J_DOF;i++){ q_old(i) = m_robot->joint(i)->q; }
      int loop;
      for(loop=0; loop < _max_iteration;){
        solveFullbodyIK(m_robot, _ik_tgt_list);
        //check ang moment
        m_robot->rootLink()->v = (m_robot->rootLink()->p - base_p_old)/ m_dt;
        m_robot->rootLink()->w = base_R_old * omegaFromRotEx(base_R_old.transpose() * m_robot->rootLink()->R) / m_dt;
        for(int i=0;i<J_DOF;i++){ m_robot->joint(i)->dq = (m_robot->joint(i)->q - q_old(i)) / m_dt; }
        m_robot->calcForwardKinematics(true,false);
        m_robot->calcTotalMomentum(cur_P, cur_L);
        loop++;
        if(checkIKConvergence(_ik_tgt_list)){ break; }
      }
      return loop;
    }
    void solveFullbodyIK (const hrp::Vector3& _dif_cog, const bool _is_transition){
      SimpleFullbodyInverseKinematicsSolver::solveFullbodyIK (_dif_cog, _is_transition);
    }
    bool checkIKConvergence(const std::vector<IKConstraintParam>& _ik_tgt_list){
      for ( int i=0; i<_ik_tgt_list.size(); i++ ) {
        hrp::Link* target_link_ptr = m_robot->link(_ik_tgt_list[i].target_link_name);
        if(target_link_ptr){
          hrp::Vector3 pos_err = _ik_tgt_list[i].targetPos - (target_link_ptr->p + target_link_ptr->R * _ik_tgt_list[i].localPos);
          hrp::Vector3 rot_err = omegaFromRotEx(_ik_tgt_list[i].targetR.transpose() * target_link_ptr->R * _ik_tgt_list[i].localR);
          for(int j=0;j<3;j++){
            if(pos_err(j) > _ik_tgt_list[i].pos_precision && _ik_tgt_list[i].selection_vec(j) != 0){return false;}
            if(rot_err(j) > _ik_tgt_list[i].rot_precision && _ik_tgt_list[i].selection_vec(j+3) != 0){return false;}
          }
        }
        else if(!target_link_ptr && _ik_tgt_list[i].target_link_name == "COM"){  // COM
          hrp::Vector3 pos_err = _ik_tgt_list[i].targetPos - (m_robot->calcCM() + _ik_tgt_list[i].localR * _ik_tgt_list[i].localPos);
//            robot_in->calcTotalMomentum(fik_in->cur_P, fik_in->cur_L);
//            hrp::Vector3 rot_err = omegaFromRotEx(ik_tgt_list[i].target_r.transpose() * ik_tgt_list[i].localR * target_link_ptr->R);
          for(int j=0;j<3;j++){
            if(pos_err(j) > _ik_tgt_list[i].pos_precision && _ik_tgt_list[i].selection_vec(j) != 0){return false;}
//              if(rot_err(j) > ik_tgt_list[i].rot_precision && ik_tgt_list[i].selection_vec(j+3) != 0){return false;}
          }
        }
      }
      return true;
    }
    void solveFullbodyIK(hrp::BodyPtr& _robot, const std::vector<IKConstraintParam>& _ik_tgt_list){
      J_all = hrp::dmatrix::Zero(WS_DOF*_ik_tgt_list.size(), ALL_DOF);
      dq_all.fill(0);
      dp_ee_all = hrp::dvector::Zero(WS_DOF*_ik_tgt_list.size());
      selection_vec_all = hrp::ivector::Zero(WS_DOF*_ik_tgt_list.size());
      weight_vec_all = hrp::dvector::Ones(WS_DOF*_ik_tgt_list.size());
      //リファレンスに微少量戻す
      for(int i=0;i<J_DOF;i++){ _robot->joint(i)->q = _robot->joint(i)->q * (1-reference_gain(i)) + reference_q(i) * reference_gain(i); }
      _robot->rootLink()->p = _robot->rootLink()->p.cwiseProduct(hrp::Vector3::Ones()-reference_gain.segment(J_DOF,3)) + reference_q.segment(J_DOF,3).cwiseProduct(reference_gain.segment(J_DOF,3));
      _robot->rootLink()->R = hrp::rotFromRpy( hrp::rpyFromRot(_robot->rootLink()->R).cwiseProduct(hrp::Vector3::Ones()-reference_gain.segment(J_DOF+3,3)) + reference_q.segment(J_DOF+3,3).cwiseProduct(reference_gain.segment(J_DOF+3,3)));
      for(int i=0;i<J_DOF;i++){ LIMIT_MINMAX(_robot->joint(i)->q, _robot->joint(i)->llimit, _robot->joint(i)->ulimit); }
      _robot->calcForwardKinematics();
      //ヤコビアンと各ベクトル生成
      for ( int i=0; i<_ik_tgt_list.size(); i++ ) {
        hrp::Link* target_link_ptr = _robot->link(_ik_tgt_list[i].target_link_name);
        hrp::Matrix33 ref_link_origin_R(_ik_tgt_list[i].targetR * _ik_tgt_list[i].localR.transpose()); //拘束ポイントの位置姿勢->リンク原点の位置姿勢
        hrp::Vector3 ref_link_origin_p(_ik_tgt_list[i].targetPos - ref_link_origin_R * _ik_tgt_list[i].localPos);
        hrp::dmatrix J_part = hrp::dmatrix::Zero(WS_DOF, ALL_DOF); //全身to一拘束点へのヤコビアン
        hrp::Vector3 vel_p_ref, vel_r_ref; //velと言いつつ実際は差分
        //ベースリンク，通常リンク，重心で場合分け
        if(target_link_ptr){//ベースリンク，通常リンク共通
          vel_p_ref = ref_link_origin_p - target_link_ptr->p;
          vel_r_ref = target_link_ptr->R * omegaFromRotEx(target_link_ptr->R.transpose() * ref_link_origin_R);

          if(target_link_ptr == _robot->rootLink()){ //ベース限定
            J_part.rightCols(WS_DOF) = hrp::dmatrix::Identity(WS_DOF, WS_DOF);
          }
          else{ //通常リンク限定
            hrp::JointPathEx tgt_jpath(_robot, _robot->rootLink(), target_link_ptr, m_dt, false, "");
            hrp::dmatrix J_jpath;
            tgt_jpath.calcJacobian(J_jpath);
            for(int id_in_jpath=0; id_in_jpath<tgt_jpath.numJoints(); id_in_jpath++){ //ジョイントパスのJaxobianを全身用に並び替え
              int id_in_body = tgt_jpath.joint(id_in_jpath)->jointId; //全身でのjoint番号
              J_part.col(id_in_body) = J_jpath.col(id_in_jpath);
            }
            J_part.rightCols(BASE_DOF) = hrp::dmatrix::Identity( WS_DOF,  BASE_DOF );
            J_part.rightCols(BASE_DOF).topRightCorner(3,3) = hrp::hat(target_link_ptr->p - _robot->rootLink()->p);
          }
        }
        else if(!target_link_ptr && _ik_tgt_list[i].target_link_name == "COM"){ //重心限定
          vel_p_ref = ref_link_origin_p - _robot->calcCM();
          vel_r_ref = (omegaFromRotEx(ref_link_origin_R) - (cur_L - _robot->rootLink()->p.cross(cur_P))) * m_dt;// COMのrotはAngulerMomentumとして扱う
          _robot->calcCMJacobian(NULL, J_com);//デフォで右端に3x6のbase->COMのヤコビアンが付いてくる
          _robot->calcAngularMomentumJacobian(NULL, J_am);//すでにrootlink周りの角運動量ヤコビアンが返ってくる？
          J_part << J_com, J_am;
        }
        else{ std::cerr<<"Unknown Link Target !!"<<std::endl; continue; } //不明なリンク指定
        //全体の中に配置
        if(vel_p_ref.norm()>0.1){vel_p_ref = vel_p_ref.normalized() * 0.1;}
        if(vel_r_ref.norm()>0.1){vel_r_ref = vel_r_ref.normalized() * 0.1;}
        dp_ee_all.segment(WS_DOF*i,WS_DOF) << vel_p_ref, vel_r_ref;
        weight_vec_all.segment(WS_DOF*i, WS_DOF) = _ik_tgt_list[i].weight_vec;
        selection_vec_all.segment(WS_DOF*i, WS_DOF) = _ik_tgt_list[i].selection_vec;
        J_all.middleRows(WS_DOF*i, WS_DOF) = J_part;
      }

      hrp::dmatrix selection_matrix = hrp::dmatrix::Zero(selection_vec_all.sum(), WS_DOF*_ik_tgt_list.size());
      for(int i=0, j=0; i<selection_vec_all.rows(); i++){ if(selection_vec_all(i) == 1){ selection_matrix(j, i) = 1.0; j++; } }

      weight_vec_all /= weight_vec_all.maxCoeff();
      J_all = selection_matrix * J_all;
      dp_ee_all = selection_matrix * weight_vec_all.asDiagonal() * dp_ee_all;

//      hrp::calcPseudoInverse(J_all,J_all_inv,1.0e-3);//超遅い1[ms]とか
      calcJacobianInverseNullspace_copy(_robot, J_all, J_all_inv);

      dq_all = J_all_inv * dp_ee_all; // dq = pseudoInverse(J) * v
      static int count;
      if(count++ % 10000 == 0){
        std::cout<<std::setprecision(2) << "J=\n"<<J_all<<std::setprecision(6)<<std::endl;
        std::cout<<std::setprecision(2) << "J_all_inv=\n"<<J_all_inv<<std::setprecision(6)<<std::endl;
        std::cout<<std::setprecision(2) << "selection_matrix=\n"<<selection_matrix<<std::setprecision(6)<<std::endl;
        std::cout<<"eevel_all\n"<<dp_ee_all.transpose()<<std::endl;
        std::cout<<"dq_all\n"<<dq_all.transpose()<<std::endl;
        std::cout<<"q_ans_all\n";
        for(int i=0;i<_robot->numJoints();i++)std::cerr<<_robot->joint(i)->q<<" ";
        std::cout<<std::endl;
      }

      const double LAMBDA = 1.0;
      // check validity
      hrp::Matrix33 base_R_ans = hrp::hat(dq_all.segment(J_DOF+3,3) * LAMBDA).exp() * _robot->rootLink()->R;
      if(!base_R_ans.isUnitary()){ std::cerr <<"ERROR base_R_ans is not Unitary" << std::endl; return; }
      for(int i=0;i<dq_all.rows();i++){ if( isnan(dq_all(i)) || isinf(dq_all(i)) ){ std::cerr <<"ERROR nan/inf is found" << std::endl; return;} }

      //関節角+ベース位置姿勢更新
      for(int i=0;i<J_DOF;i++){
        _robot->joint(i)->q += dq_all(i) * LAMBDA/* * m_dt*/;
        LIMIT_MINMAX(_robot->joint(i)->q, _robot->joint(i)->llimit, _robot->joint(i)->ulimit);
      }
      _robot->rootLink()->p += dq_all.segment(J_DOF,3) * LAMBDA/* * m_dt*/;//vel計算する時からm_dt使ってないから使わなくていい
      _robot->rootLink()->R = base_R_ans;
    }


    bool calcJacobianInverseNullspace_copy(hrp::BodyPtr _robot, hrp::dmatrix &_J, hrp::dmatrix &_Jinv) {
      hrp::dmatrix w = hrp::dmatrix::Identity(ALL_DOF,ALL_DOF);
      std::vector<double> avoid_weight_gain;
      avoid_weight_gain.resize(_robot->numJoints());
      for(int j=0; j < J_DOF; j++){
        avoid_weight_gain[j] = 100000000000000000000.0;
      }
      bool use_inside_joint_weight_retrieval = true;

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
        if (( r - avoid_weight_gain[j] ) >= 0 ) {
          w(j, j) = optional_weight_vector[j] * ( 1.0 / ( 1.0 + r) );
        } else {
          if (use_inside_joint_weight_retrieval)
          w(j, j) = optional_weight_vector[j] * 1.0;
          else
          w(j, j) = optional_weight_vector[j] * ( 1.0 / ( 1.0 + r) );
        }
        avoid_weight_gain[j] = r;
      }
      for ( int j = _robot->numJoints(); j < ALL_DOF ; j++ ) {
        w(j, j) = optional_weight_vector[j];
      }

      const double manipulability_limit = 0.001;
      const double sr_gain = 1.0;
//      const double manipulability_gain = 0.001;
      const double manipulability_gain = 1;

      double manipulability = sqrt((_J*_J.transpose()).determinant());
      double k = 0;
      if ( manipulability < manipulability_limit ) {
        k = manipulability_gain * pow((1 - ( manipulability / manipulability_limit )), 2);
      }
      hrp::calcSRInverse(_J, _Jinv, sr_gain * k, w);
      //        Jnull = ( hrp::dmatrix::Identity(n, n) - _Jinv * _J);
      return true;
    }
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


class FullbodyInverseKinematicsSolverMT : public FullbodyInverseKinematicsSolver{
  protected:
    hrp::BodyPtr m_robot_copy;
    pthread_t ik_thread;
    pthread_mutex_t ik_body_mutex;  // Mutex
    int cur_ik_loop;
    std::vector<IKConstraintParam> ik_tgt_list;
    bool is_first;
    bool ik_thread_kill;
    bool ik_thread_ik_required;
    hrp::Vector3 base_p_old;
    hrp::Matrix33 base_R_old;
    hrp::dvector q_old;
  public:
    FullbodyInverseKinematicsSolverMT (hrp::BodyPtr& _robot, const std::string& _print_str, const double _dt) : FullbodyInverseKinematicsSolver(_robot,_print_str, _dt) {
      m_robot_copy = hrp::BodyPtr(new hrp::Body(*_robot));
      cur_ik_loop = 0;
      is_first = true;
      ik_thread_kill = false;
      ik_thread_ik_required = false;
    };
    ~FullbodyInverseKinematicsSolverMT () {
      pthread_cancel(ik_thread);
      ik_thread_kill = true;
      pthread_join(ik_thread, NULL );
      std::cerr<<"[FullbodyInverseKinematicsSolverMT] thread killed successfully"<<std::endl;
    };
    int solveFullbodyIKLoopMT (const std::vector<IKConstraintParam>& _ik_tgt_list, const int _max_iteration) {
      if(is_first){
        for(int i=0;i<J_DOF;i++){ m_robot_copy->joint(i)->q = m_robot->joint(i)->q; }
        m_robot_copy->rootLink()->p = m_robot->rootLink()->p;
        m_robot_copy->rootLink()->R = m_robot->rootLink()->R;
        if( m_robot_copy->link("RARM_JOINT2") != NULL) m_robot_copy->link("RARM_JOINT2")->ulimit = deg2rad(-40);//脇の干渉回避のため
        if( m_robot_copy->link("LARM_JOINT2") != NULL) m_robot_copy->link("LARM_JOINT2")->llimit = deg2rad(40);
        pthread_mutex_init( &ik_body_mutex, NULL );
      }
      for(int i=0;i<J_DOF;i++){ m_robot->joint(i)->q = m_robot_copy->joint(i)->q; }
      m_robot->rootLink()->p = m_robot_copy->rootLink()->p;
      m_robot->rootLink()->R = m_robot_copy->rootLink()->R;
      int result = cur_ik_loop;

      if(pthread_mutex_lock( &ik_body_mutex ) != 0){std::cerr<<"[FullbodyInverseKinematicsSolverMT] pthread_mutex_lock err "<<std::endl;}

//        base_p_old = m_robot_copy->rootLink()->p;
//        base_R_old = m_robot_copy->rootLink()->R;
//        q_old.resize(J_DOF);
//        for(int i=0;i<J_DOF;i++){ q_old(i) = m_robot_copy->joint(i)->q; }

        cur_ik_loop = 0;
        ik_tgt_list = _ik_tgt_list;// "="で代入するとnanが入る！？->そんなことはない
        ik_thread_ik_required = true;
      if(pthread_mutex_unlock( &ik_body_mutex ) != 0){std::cerr<<"[FullbodyInverseKinematicsSolverMT] pthread_mutex_unlock err "<<std::endl;}
      if(is_first){
        pthread_create(&ik_thread, NULL, launchThread, this);
        is_first = false;
      }

      return result;
    }
    void ik_loop(){
      std::cerr<<"[FullbodyInverseKinematicsSolverMT] start thread"<<std::endl;
      while(1){
        usleep(0);//これ無いとmutex解放しない？
        if(pthread_mutex_lock( &ik_body_mutex ) != 0){std::cerr<<"[FullbodyInverseKinematicsSolverMT] pthread_mutex_lock2 err "<<std::endl;}
          if(ik_thread_ik_required){
            solveFullbodyIK(m_robot_copy, ik_tgt_list);
            //check ang moment
//            m_robot_copy->rootLink()->v = (m_robot_copy->rootLink()->p - base_p_old)/ m_dt;
//            m_robot_copy->rootLink()->w = base_R_old * omegaFromRotEx(base_R_old.transpose() * m_robot_copy->rootLink()->R) / m_dt;
//            for(int i=0;i<J_DOF;i++){ m_robot_copy->joint(i)->dq = (m_robot_copy->joint(i)->q - q_old(i)) / m_dt; }
//            m_robot_copy->calcForwardKinematics(true,false);
//            m_robot_copy->calcTotalMomentum(cur_P, cur_L);
            cur_ik_loop++;
            if(cur_ik_loop > 10000){std::cerr<<"[FullbodyInverseKinematicsSolverMT] thread time out"<<std::endl;break;}//異常終了でスレッドが残ってしまった時にタイムアウト終了
          }
        if(pthread_mutex_unlock( &ik_body_mutex ) != 0){std::cerr<<"[FullbodyInverseKinematicsSolverMT] pthread_mutex_unlock2 err "<<std::endl;}
        if(ik_thread_kill){break;}
      }
      std::cerr<<"[FullbodyInverseKinematicsSolverMT] end of thread"<<std::endl;
    }
    static void* launchThread(void *pParam) {
      reinterpret_cast<FullbodyInverseKinematicsSolverMT*>(pParam)->ik_loop();
      return NULL;
    }
};




#endif //  FULLBODYIK_H
