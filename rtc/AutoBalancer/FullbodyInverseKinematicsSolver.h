#ifndef FULLBODYIK_H
#define FULLBODYIK_H

#include <float.h>
#include <hrpModel/Body.h>
#include <hrpUtil/MatrixSolvers.h>
#include "../ImpedanceController/JointPathEx.h"
#include "SimpleFullbodyInverseKinematicsSolver.h"

#define LIMIT_MINMAX(x,min,max) ((x= (x<min  ? min : x<max ? x : max)))
#define deg2rad(x)((x)*M_PI/180)
#define rad2deg(rad) (rad * 180 / M_PI)
#define eps_eq(a, b, c)  (fabs((a)-(b)) <= c)


class IKConstraintParam {
  public:
    std::string target_link_name;
    // IK target EE coords
    hrp::Vector3 target_p0;
    hrp::Matrix33 target_r0;
    // EE offset, EE link
    hrp::Vector3 localPos;
    hrp::Matrix33 localR;
    hrp::dvector6 weight_vec;
    hrp::ivector selection_vec;

    IKConstraintParam ()
      :target_p0(hrp::Vector3::Zero()),
       target_r0(hrp::Matrix33::Identity()),
       localPos(hrp::Vector3::Zero()),
       localR(hrp::Matrix33::Identity()),
       weight_vec(hrp::dvector6::Ones()),
       selection_vec(hrp::ivector::Ones(6)){};
};


class FullbodyInverseKinematicsSolver : public SimpleFullbodyInverseKinematicsSolver{
  protected:
    hrp::dmatrix J_all, J_all_inv;
    hrp::dvector dq_all, dp_ee_all;
    hrp::dmatrix J_jpath;
    hrp::dmatrix J_com, J_am;

  public:
    std::vector<IKConstraintParam> ik_tgt_list;
    hrp::Vector3 target_com_p0;
    hrp::Matrix33 target_com_r0;
    hrp::dvector optional_weight_vector;
    FullbodyInverseKinematicsSolver (hrp::BodyPtr& _robot, const std::string& _print_str, const double _dt) : SimpleFullbodyInverseKinematicsSolver(_robot,_print_str, _dt) {
        qorg.resize(m_robot->numJoints());
        qrefv.resize(m_robot->numJoints());
        optional_weight_vector = hrp::dvector::Ones(m_robot->numJoints()+6);
    };
    hrp::Vector3 cur_P,cur_L;
    ~FullbodyInverseKinematicsSolver () {};
    // Solve fullbody IK using limb IK
    void solveFullbodyIK ( const hrp::Vector3& _dif_cog, const bool is_transition) {

      solveIK();

    }
    void solveIK (){
      const int BASE_DOF = 6;
      const int WS_DOF = 6;
      const int ALL_DOF = BASE_DOF + m_robot->numJoints();

      m_robot->calcForwardKinematics();
      J_all = hrp::dmatrix::Zero(WS_DOF*ik_tgt_list.size(), ALL_DOF);
      dq_all = hrp::dvector::Zero( ALL_DOF );
      dp_ee_all = hrp::dvector::Zero(WS_DOF*ik_tgt_list.size());
      hrp::ivector selection_vec_all = hrp::ivector::Zero(WS_DOF*ik_tgt_list.size());
      hrp::dvector weight_vec_all = hrp::dvector::Ones(WS_DOF*ik_tgt_list.size());

      for ( int i=0; i<ik_tgt_list.size(); i++ ) {
        hrp::Link* target_link_ptr = m_robot->link(ik_tgt_list[i].target_link_name);
        hrp::Matrix33 ref_link_origin_R(ik_tgt_list[i].target_r0 * ik_tgt_list[i].localR.transpose());
        hrp::Vector3 ref_link_origin_p(ik_tgt_list[i].target_p0 - ref_link_origin_R * ik_tgt_list[i].localPos);
        hrp::dmatrix J_part = hrp::dmatrix::Zero(WS_DOF, ALL_DOF);

        if(target_link_ptr){
          hrp::Vector3 vel_p_ref(ref_link_origin_p - target_link_ptr->p);
          hrp::Vector3 vel_r_ref(target_link_ptr->R * omegaFromRotEx(target_link_ptr->R.transpose() * ref_link_origin_R));
          dp_ee_all.segment(WS_DOF*i,WS_DOF) << vel_p_ref , vel_r_ref;

          if(target_link_ptr == m_robot->rootLink()){      //base specific
            J_part.rightCols(WS_DOF) = hrp::dmatrix::Identity(WS_DOF, WS_DOF);
          }
          else{    //normal link
            hrp::JointPathEx tgt_jpex(m_robot, m_robot->rootLink(), target_link_ptr, m_dt, false, "");
            tgt_jpex.calcJacobian(J_jpath);
            for(int id_in_jpath=0; id_in_jpath<tgt_jpex.numJoints(); id_in_jpath++){
              int id_in_body = tgt_jpex.joint(id_in_jpath)->jointId; //全身でのjoint番号
              J_part.col(id_in_body) = J_jpath.col(id_in_jpath);
            }
            J_part.rightCols(BASE_DOF) = hrp::dmatrix::Identity( WS_DOF,  BASE_DOF );
            J_part.rightCols(BASE_DOF).topRightCorner(3,3) = hrp::hat(target_link_ptr->p - m_robot->rootLink()->p);
          }
        }
        else if(!target_link_ptr && ik_tgt_list[i].target_link_name == "COM"){  // COM
          hrp::Vector3 vel_com_p_ref(ref_link_origin_p - m_robot->calcCM());
//          hrp::Vector3 vel_com_r_ref = (omegaFromRotEx(ref_link_origin_R) - cur_L) * m_dt;// COMのrotはAngulerMomentumとして扱う
          hrp::Vector3 vel_com_r_ref = (omegaFromRotEx(ref_link_origin_R) - (cur_L - m_robot->rootLink()->p.cross(cur_P))) * m_dt;// COMのrotはAngulerMomentumとして扱う
//          std::cerr<<"cur_L "<<cur_L.transpose()<<std::endl;
//          std::cerr<<"m_robot->rootLink()->p.cross(cur_P) "<<m_robot->rootLink()->p.cross(cur_P).transpose()<<std::endl;
//          std::cerr<<"vel_com_r_ref "<<vel_com_r_ref.transpose()<<std::endl;
          dp_ee_all.segment(WS_DOF*i,WS_DOF) << vel_com_p_ref, vel_com_r_ref;
          m_robot->calcCMJacobian(NULL, J_com);//デフォで右端に3x6のbase->COMのヤコビアンが付いてくる
          m_robot->calcAngularMomentumJacobian(NULL, J_am);//すでにrootlink周りの角運動量ヤコビアンが返ってくる？
          J_part << J_com, J_am;
        }
        else{// Error
          std::cerr<<"Unknown Link Target !!"<<std::endl; continue;
        }
        weight_vec_all.segment(WS_DOF*i, WS_DOF) = ik_tgt_list[i].weight_vec;
        selection_vec_all.segment(WS_DOF*i, WS_DOF) = ik_tgt_list[i].selection_vec;
        J_all.middleRows(WS_DOF*i, WS_DOF) = J_part;
      }

      hrp::dmatrix selection_matrix = hrp::dmatrix::Zero(selection_vec_all.sum(), WS_DOF*ik_tgt_list.size());
      for(int i=0, j=0; i<selection_vec_all.rows(); i++){ if(selection_vec_all(i) == 1){ selection_matrix(j, i) = 1.0; j++; } }

      weight_vec_all /= weight_vec_all.maxCoeff();
      J_all = selection_matrix * J_all;
      dp_ee_all = selection_matrix * weight_vec_all.asDiagonal() * dp_ee_all;

//      hrp::calcPseudoInverse(J_all,J_all_inv,1.0e-3);//超遅い1[ms]とか
      calcJacobianInverseNullspace_copy(m_robot, J_all, J_all_inv);

      static int count;

      dq_all = J_all_inv * dp_ee_all; // dq = pseudoInverse(J) * v
      if(count % 2000 == 0){
        std::cout<<"solveIK"<<std::endl;
        std::cout<<"eevel_all\n"<<dp_ee_all<<std::endl;
        std::cout<<std::setprecision(2) << "J=\n"<<J_all<<std::setprecision(6)<<std::endl;
        std::cout<<std::setprecision(2) << "J_all_inv=\n"<<J_all_inv<<std::setprecision(6)<<std::endl;
        std::cout<<std::setprecision(2) << "selection_matrix=\n"<<selection_matrix<<std::setprecision(6)<<std::endl;
        std::cout<<"dq_all\n"<<dq_all<<std::endl;
      }

      const double LAMBDA = 1.0;

      m_robot->rootLink()->p += dq_all.segment(m_robot->numJoints(),3) * LAMBDA/* * m_dt*/;//vel計算する時からm_dt使ってないから使わなくていい
      m_robot->rootLink()->R *= hrp::rotFromRpy( dq_all.segment(m_robot->numJoints()+3,3)) * LAMBDA /* * m_dt*/ ;
//
//      m_robot->rootLink()->v = dq_all.segment(m_robot->numJoints(),3) * LAMBDA / m_dt; //for ang moment
//      m_robot->rootLink()->w = dq_all.segment(m_robot->numJoints()+3,3) * LAMBDA / m_dt; //for ang moment

      for(int i=0;i<m_robot->numJoints();i++){
        m_robot->joint(i)->q += dq_all(i) * LAMBDA/* * m_dt*/;
        LIMIT_MINMAX(m_robot->joint(i)->q, m_robot->joint(i)->llimit, m_robot->joint(i)->ulimit);
//        m_robot->joint(i)->dq = dq_all(i) * LAMBDA / m_dt; //for calc ang moment
      }
      count++;
    }


    bool calcJacobianInverseNullspace_copy(hrp::BodyPtr robot, hrp::dmatrix &J, hrp::dmatrix &Jinv) {
      const int BASE_DOF = 6;
      const int ALL_DOF = J.cols();//BASE_DOF + m_robot->numJoints()
      hrp::dmatrix w = hrp::dmatrix::Identity(ALL_DOF,ALL_DOF);
      std::vector<double> avoid_weight_gain;
      avoid_weight_gain.resize(robot->numJoints());
      for(int j=0; j < m_robot->numJoints(); j++){
        avoid_weight_gain[j] = 100000000000000000000.0;
      }
      bool use_inside_joint_weight_retrieval = true;

      for ( int j = 0; j < robot->numJoints() ; j++ ) {
        double jang = robot->joint(j)->q;
        double jmax = robot->joint(j)->ulimit;
        double jmin = robot->joint(j)->llimit;
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

      const double manipulability_limit = 0.1;
      const double sr_gain = 1.0;
      const double manipulability_gain = 0.001;

      double manipulability = sqrt((J*J.transpose()).determinant());
      double k = 0;
      if ( manipulability < manipulability_limit ) {
        k = manipulability_gain * pow((1 - ( manipulability / manipulability_limit )), 2);
      }
      hrp::calcSRInverse(J, Jinv, sr_gain * k, w);
      //        Jnull = ( hrp::dmatrix::Identity(n, n) - Jinv * J);
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


#endif //  FULLBODYIK_H
