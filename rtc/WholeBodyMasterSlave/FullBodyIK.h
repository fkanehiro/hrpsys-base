#ifndef FULLBODYIK_H
#define FULLBODYIK_H

#include <hrpModel/Body.h>
#include <hrpUtil/MatrixSolvers.h>
#include "../ImpedanceController/JointPathEx.h"
#include "../ImpedanceController/RatsMatrix.h"

#include "../AutoBalancer/SimpleFullbodyInverseKinematicsSolver.h"

#define LIMIT_MINMAX(x,min,max) ((x= (x<min  ? min : x<max ? x : max)))


class FullbodyInverseKinematicsSolver : public SimpleFullbodyInverseKinematicsSolver{
  protected:
    hrp::dmatrix J_all, J_all_inv;
    hrp::dvector dq_all, eevel_all;

  public:
    hrp::Vector3 target_com_p0;
    hrp::Matrix33 target_com_r0;
    FullbodyInverseKinematicsSolver (hrp::BodyPtr& _robot, const std::string& _print_str, const double _dt) : SimpleFullbodyInverseKinematicsSolver(_robot,_print_str, _dt)
    {
        qorg.resize(m_robot->numJoints());
        qrefv.resize(m_robot->numJoints());
        limb_stretch_avoidance_vlimit[0] = -1000 * 1e-3 * _dt; // lower limit
        limb_stretch_avoidance_vlimit[1] = 50 * 1e-3 * _dt; // upper limit
    };
    ~FullbodyInverseKinematicsSolver () {};

    // Solve fullbody IK using limb IK
    void solveFullbodyIK (const hrp::Vector3& _dif_cog, const bool is_transition)
    {
        m_robot->calcForwardKinematics();
        solveIK();
        m_robot->calcForwardKinematics();
    }
    hrp::Vector3 matrix_logEx(const hrp::Matrix33& m) {// copy
        hrp::Vector3 mlog;
        double q0, th;
        hrp::Vector3 q;
        double norm;

        Eigen::Quaternion<double> eiq(m);
        q0 = eiq.w();
        q = eiq.vec();
        norm = q.norm();
        if (norm > 0) {
            if ((q0 > 1.0e-10) || (q0 < -1.0e-10)) {
                th = 2 * std::atan(norm / q0);
            } else if (q0 > 0) {
                th = M_PI / 2;
            } else {
                th = -M_PI / 2;
            }
            mlog = (th / norm) * q ;
        } else {
            mlog = hrp::Vector3::Zero();
        }
        return mlog;
    }
#define USE_BASE_CONSTRAINT 0
    void solveIK (){
      const int BASE_DOF = 6;
      const int COM_DOF = 6;
      const int WS_DOF = 6;
      const int eenum = ikp.size();
      const int ALL_DOF = BASE_DOF + m_robot->numJoints();

      m_robot->calcForwardKinematics();
      J_all = hrp::dmatrix::Zero( BASE_DOF+WS_DOF*eenum+COM_DOF, ALL_DOF );
      J_all_inv = hrp::dmatrix::Zero( ALL_DOF,  BASE_DOF+WS_DOF*eenum+COM_DOF );
      dq_all = hrp::dvector::Zero( ALL_DOF );
      eevel_all = hrp::dvector::Zero( BASE_DOF+WS_DOF*eenum+COM_DOF );

      int ee_num_count = 0;
      for ( std::map<std::string, IKparam>::iterator it = ikp.begin(); it != ikp.end(); it++ ) {
        hrp::Matrix33 target_link_R(it->second.target_r0 * it->second.localR.transpose());
        hrp::Vector3 target_link_p(it->second.target_p0 - target_link_R * it->second.localPos);
        hrp::Vector3 vel_p(target_link_p - it->second.manip->endLink()->p);
        hrp::Vector3 vel_r(it->second.manip->endLink()->R * matrix_logEx(it->second.manip->endLink()->R.transpose() * target_link_R));
        eevel_all.segment( BASE_DOF+ee_num_count*WS_DOF, WS_DOF) << vel_p , vel_r;
        ee_num_count++;
      }

      const double com_ik_weight = 1;
      hrp::Vector3 vel_com_p(target_com_p0 - m_robot->calcCM());
      hrp::Vector3 vel_com_r(m_robot->rootLink()->R * matrix_logEx(m_robot->rootLink()->R.transpose() * target_com_r0));
      eevel_all.segment( BASE_DOF+eenum*WS_DOF, WS_DOF) << vel_com_p * com_ik_weight , vel_com_r * com_ik_weight;

      J_all.block(0, 0, WS_DOF, WS_DOF ) = hrp::dmatrix::Identity(WS_DOF, WS_DOF);

      int ee_num_count2 = 0;
      for ( std::map<std::string, IKparam>::iterator it = ikp.begin(); it != ikp.end(); it++ ) {
        hrp::dmatrix J_b2ee = hrp::dmatrix::Identity( WS_DOF,  BASE_DOF );
        hrp::Vector3 p_b2ee = it->second.manip->endLink()->p - m_robot->rootLink()->p;
        J_b2ee.block(0, 3, 3, 1) = hrp::Vector3::UnitX().cross(p_b2ee);
        J_b2ee.block(0, 4, 3, 1) = hrp::Vector3::UnitY().cross(p_b2ee);
        J_b2ee.block(0, 5, 3, 1) = hrp::Vector3::UnitZ().cross(p_b2ee);
        J_all.block(BASE_DOF+WS_DOF*ee_num_count2, 0, WS_DOF, WS_DOF ) = J_b2ee;
        hrp::dmatrix J_ee;
        it->second.manip->calcJacobian(J_ee);
        for(int id_in_limb=0; id_in_limb<it->second.manip->numJoints(); id_in_limb++){
          int id_in_body = it->second.manip->joint(id_in_limb)->jointId; //全身でのjoint番号
          J_all.block(BASE_DOF+ee_num_count2*WS_DOF, BASE_DOF+id_in_body, 6, 1) = J_ee.col(id_in_limb);
        }
        ee_num_count2++;
      }

      hrp::dmatrix J_com;
      m_robot->calcCMJacobian(NULL, J_com);
      J_all.block(BASE_DOF+eenum*WS_DOF, BASE_DOF, 3, m_robot->numJoints()) = J_com;

      hrp::dmatrix J_b2com = hrp::dmatrix::Identity( COM_DOF,  BASE_DOF );
      hrp::Vector3 p_b2com = m_robot->calcCM() - m_robot->rootLink()->p;
      J_b2com.block(0, 3, 3, 1) = hrp::Vector3::UnitX().cross(p_b2com);
      J_b2com.block(0, 4, 3, 1) = hrp::Vector3::UnitY().cross(p_b2com);
      J_b2com.block(0, 5, 3, 1) = hrp::Vector3::UnitZ().cross(p_b2com);
      J_all.block(BASE_DOF+eenum*WS_DOF, 0, COM_DOF, BASE_DOF) = J_b2com;

//      //base拘束しない場合
      if(!USE_BASE_CONSTRAINT){
        eevel_all = eevel_all.bottomRows(WS_DOF*eenum+COM_DOF);//上6要素削除
        J_all = J_all.bottomRows(WS_DOF*eenum+COM_DOF);//上6行削除
      }

      struct timespec startT, endT;
      clock_gettime(CLOCK_REALTIME, &startT);
//      hrp::calcPseudoInverse(J_all,J_all_inv,1.0e-3);//超遅い1[ms]とか
      calcJacobianInverseNullspace_copy(m_robot, J_all, J_all_inv);

      static int count;
      if(count % 1000 == 0){clock_gettime(CLOCK_REALTIME, &endT); std::cout << (double)(endT.tv_sec - startT.tv_sec + (endT.tv_nsec - startT.tv_nsec) * 1e-9) << " @ calcPseudoInverse" << std::endl;  clock_gettime(CLOCK_REALTIME, &startT);}

      dq_all = J_all_inv * eevel_all; // dq = pseudoInverse(J) * v
      if(count % 1000 == 0){
        std::cout<<"solveIK"<<std::endl;
        std::cout<<"eevel_all\n"<<eevel_all<<std::endl;
        std::cout<<std::setprecision(2) << "J=\n"<<J_all<<std::setprecision(6)<<std::endl;
        std::cout<<std::setprecision(2) << "J_all_inv=\n"<<J_all_inv<<std::setprecision(6)<<std::endl;
        std::cout<<"dq_all\n"<<dq_all<<std::endl;
      }

      const double LAMBDA = 1.0;

      for(int i=0;i<3;i++){ m_robot->rootLink()->p(i) += dq_all(i) * LAMBDA/* * m_dt*/; }//vel計算する時からm_dt使ってないから使わなくていい
      m_robot->rootLink()->R *= hrp::rotFromRpy( hrp::Vector3(dq_all(3),dq_all(4),dq_all(5)) * LAMBDA /* * m_dt*/ );

      for(int i=0;i<m_robot->numJoints();i++){
        m_robot->joint(i)->q += dq_all(BASE_DOF + i) * LAMBDA/* * m_dt*/;
        LIMIT_MINMAX(m_robot->joint(i)->q, m_robot->joint(i)->llimit, m_robot->joint(i)->ulimit);
      }
      m_robot->calcForwardKinematics();
      count++;
    }


    bool calcJacobianInverseNullspace_copy(hrp::BodyPtr robot, hrp::dmatrix &J, hrp::dmatrix &Jinv) {
#define deg2rad(x)((x)*M_PI/180)
#define rad2deg(rad) (rad * 180 / M_PI)
#define eps_eq(a, b, c)  (fabs((a)-(b)) <= c)
      const int BASE_DOF = 6;
      const int ALL_DOF = J.cols();//BASE_DOF + m_robot->numJoints()
      hrp::dmatrix w = hrp::dmatrix::Identity(ALL_DOF,ALL_DOF);
      std::vector<double> avoid_weight_gain, optional_weight_vector;
      avoid_weight_gain.resize(robot->numJoints());
      optional_weight_vector.resize(robot->numJoints());
      for(int j=0; j < m_robot->numJoints(); j++){
        optional_weight_vector[j] = 1.0;
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
          w(BASE_DOF+j, BASE_DOF+j) = optional_weight_vector[j] * ( 1.0 / ( 1.0 + r) );
        } else {
          if (use_inside_joint_weight_retrieval)
          w(BASE_DOF+j, BASE_DOF+j) = optional_weight_vector[j] * 1.0;
          else
          w(BASE_DOF+j, BASE_DOF+j) = optional_weight_vector[j] * ( 1.0 / ( 1.0 + r) );
        }
        avoid_weight_gain[j] = r;
      }
      w.topLeftCorner(BASE_DOF, BASE_DOF) = hrp::dmatrix::Identity(BASE_DOF,BASE_DOF) * 0.1;//baseの重みテスト

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
};


#endif //  FULLBODYIK_H
