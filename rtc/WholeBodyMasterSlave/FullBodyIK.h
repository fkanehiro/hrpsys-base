#ifndef FULLBODYIK_H
#define FULLBODYIK_H

#include <hrpModel/Body.h>
#include <hrpUtil/MatrixSolvers.h>
#include "../ImpedanceController/JointPathEx.h"
#include "../ImpedanceController/RatsMatrix.h"

#include "../AutoBalancer/SimpleFullbodyInverseKinematicsSolver.h"
//#include "hrpsys/idl/AutoBalancerService.hh"
#include "../AutoBalancer/AutoBalancerService_impl.h"

class FullbodyInverseKinematicsSolver : public SimpleFullbodyInverseKinematicsSolver{
//  private:
//    // Robot model for IK
//    hrp::BodyPtr m_robot;
//    // Org (current) joint angles before IK
//    hrp::dvector qorg;
//    // IK fail checking
//    int ik_error_debug_print_freq;
//    std::string print_str;
//    bool has_ik_failed;
  public:
//    // IK parameter for each limb
//    struct IKparam {
//        // IK target EE coords
//        hrp::Vector3 target_p0;
//        hrp::Matrix33 target_r0;
//        // EE offset, EE link
//        hrp::Vector3 localPos;
//        hrp::Matrix33 localR;
//        hrp::Link* target_link;
//        // IK solver and parameter
//        hrp::JointPathExPtr manip;
//        double avoid_gain, reference_gain;
//        // IK fail checking
//        size_t pos_ik_error_count, rot_ik_error_count;
//        // Solve ik or not
//        bool is_ik_enable;
//        // Name of parent link in limb
//        std::string parent_name;
//        // Limb length
//        double max_limb_length, limb_length_margin;
//        IKparam ()
//            : avoid_gain(0.001), reference_gain(0.01),
//              pos_ik_error_count(0), rot_ik_error_count(0),
//              limb_length_margin(0.02)
//        {
//        };
//    };
//    std::map<std::string, IKparam> ikp;
//    // Used for ref joint angles overwrite before IK
//    std::vector<int> overwrite_ref_ja_index_vec;
//    // IK targets and current?
//    hrp::dvector qrefv;
//    hrp::Vector3 target_root_p;
//    hrp::Matrix33 target_root_R;
//    hrp::Vector3 current_root_p;
//    hrp::Matrix33 current_root_R;
//    // IK params
//    double move_base_gain, ratio_for_vel;
//    // For IK fail checking
//    double pos_ik_thre, rot_ik_thre;
//    // For limb stretch avoidance
//    double d_root_height, limb_stretch_avoidance_time_const, limb_stretch_avoidance_vlimit[2], m_dt;
//    bool use_limb_stretch_avoidance;

    FullbodyInverseKinematicsSolver (hrp::BodyPtr& _robot, const std::string& _print_str, const double _dt) : SimpleFullbodyInverseKinematicsSolver(_robot,_print_str, _dt)
//        : m_robot(_robot),
//          ik_error_debug_print_freq(static_cast<int>(0.2/_dt)), // once per 0.2 [s]
//          has_ik_failed(false),
//          overwrite_ref_ja_index_vec(), ikp(),
//          move_base_gain(0.8), ratio_for_vel(1.0),
//          pos_ik_thre(0.5*1e-3), // [m]
//          rot_ik_thre((1e-2)*M_PI/180.0), // [rad]
//          print_str(_print_str), m_dt(_dt),
//          use_limb_stretch_avoidance(false), limb_stretch_avoidance_time_const(1.5)
    {
        qorg.resize(m_robot->numJoints());
        qrefv.resize(m_robot->numJoints());
        limb_stretch_avoidance_vlimit[0] = -1000 * 1e-3 * _dt; // lower limit
        limb_stretch_avoidance_vlimit[1] = 50 * 1e-3 * _dt; // upper limit
    };
    ~FullbodyInverseKinematicsSolver () {};

    void initializeInterlockingJoints (std::vector<std::pair<hrp::Link*, hrp::Link*> > & interlocking_joints)
    {
        for ( std::map<std::string, IKparam>::iterator it = ikp.begin(); it != ikp.end(); it++ ) {
            std::cerr << "[" << print_str << "] Interlocking Joints for [" << it->first << "]" << std::endl;
            it->second.manip->setInterlockingJointPairIndices(interlocking_joints, print_str);
        }
    };
    void storeCurrentParameters()
    {
        current_root_p = m_robot->rootLink()->p;
        current_root_R = m_robot->rootLink()->R;
        for ( unsigned int i = 0; i < m_robot->numJoints(); i++ ){
            qorg[i] = m_robot->joint(i)->q;
        }
    };
    void revertRobotStateToCurrent ()
    {
        for ( std::map<std::string, IKparam>::iterator it = ikp.begin(); it != ikp.end(); it++ ) {
            if (it->second.is_ik_enable) {
                for ( unsigned int j = 0; j < it->second.manip->numJoints(); j++ ){
                    int i = it->second.manip->joint(j)->jointId;
                    m_robot->joint(i)->q = qorg[i];
                }
            }
        }
        m_robot->rootLink()->p = current_root_p;
        m_robot->rootLink()->R = current_root_R;
        m_robot->calcForwardKinematics();
    };
    void setReferenceJointAngles ()
    {
        for ( unsigned int i = 0; i < m_robot->numJoints(); i++ ){
            qrefv[i] = m_robot->joint(i)->q;
        }
    };
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


    void solveIK (){
      const int BASE_DOF = 6;
      const int WS_DOF = 6;
      const int eenum = ikp.size();
      const int ALL_DOF = BASE_DOF + m_robot->numJoints();
      m_robot->calcForwardKinematics();
      hrp::dmatrix J_all = hrp::dmatrix::Zero( BASE_DOF+WS_DOF*eenum, ALL_DOF );
      hrp::dmatrix J_all_inv = hrp::dmatrix::Zero( ALL_DOF,  BASE_DOF+WS_DOF*eenum );
      hrp::dvector dq_all = hrp::dvector::Zero( ALL_DOF );
      hrp::dvector eevel_all = hrp::dvector::Zero( BASE_DOF+WS_DOF*eenum );

      int ee_num_count = 0;
      for ( std::map<std::string, IKparam>::iterator it = ikp.begin(); it != ikp.end(); it++ ) {
        hrp::Matrix33 target_link_R(it->second.target_r0 * it->second.localR.transpose());
        hrp::Vector3 target_link_p(it->second.target_p0 - target_link_R * it->second.localPos);
        hrp::Vector3 vel_p(target_link_p - it->second.manip->endLink()->p);
        hrp::Vector3 vel_r(it->second.manip->endLink()->R * matrix_logEx(it->second.manip->endLink()->R.transpose() * target_link_R));
//        if(it->first == "rleg"){ eevel_all.segment(BASE_DOF, 6) << vel_p , vel_r;}
//        if(it->first == "lleg"){ eevel_all.segment(BASE_DOF+6, 6) << vel_p , vel_r;}
//        if(it->first == "rarm"){ eevel_all.segment(BASE_DOF+6+6, 6) << vel_p , vel_r;}
//        if(it->first == "larm"){ eevel_all.segment(BASE_DOF+6+6+6, 6) << vel_p , vel_r;}
        eevel_all.segment( BASE_DOF+ee_num_count*WS_DOF, WS_DOF) << vel_p , vel_r;
        ee_num_count++;
      }

      J_all.block(0, 0, WS_DOF, WS_DOF ) = hrp::dmatrix::Identity(WS_DOF, WS_DOF);

//      hrp::dmatrix tmp;
//      ikp["rleg"].manip->calcJacobian(tmp);
//      J_all.block(BASE_DOF,BASE_DOF, 6,ikp["rleg"].manip->numJoints()) = tmp;
//      ikp["lleg"].manip->calcJacobian(tmp);
//      J_all.block(BASE_DOF+6,BASE_DOF+6, 6,ikp["lleg"].manip->numJoints()) = tmp;
//      ikp["rarm"].manip->calcJacobian(tmp);
//      J_all.block(BASE_DOF+6+6,BASE_DOF+6+6+2+3, 6,ikp["rarm"].manip->numJoints()) = tmp;
//      ikp["larm"].manip->calcJacobian(tmp);
//      J_all.block(BASE_DOF+6+6+6,BASE_DOF+6+6+2+3+8, 6,ikp["larm"].manip->numJoints()) = tmp;

      int ee_num_count2 = 0;
      for ( std::map<std::string, IKparam>::iterator it = ikp.begin(); it != ikp.end(); it++ ) {
        hrp::dmatrix J_ee;
        it->second.manip->calcJacobian(J_ee);
        for(int id_in_limb=0; id_in_limb<it->second.manip->numJoints(); id_in_limb++){
          int id_in_body = it->second.manip->joint(id_in_limb)->jointId; //全身でのjoint番号
          J_all.block(BASE_DOF+ee_num_count2*WS_DOF, BASE_DOF+id_in_body, 6, 1) = J_ee.col(id_in_limb);
        }
        ee_num_count2++;
      }

      hrp::calcPseudoInverse(J_all,J_all_inv,1.0e-3);

      dq_all = J_all_inv * eevel_all; // dq = pseudoInverse(J) * v

      static int count;
      if(count++ % 1000 == 0){
        std::cout<<"eevel_all\n"<<eevel_all<<std::endl;
        std::cout<<"J\n"<<J_all<<std::endl;
        std::cout<<"J_all_inv\n"<<J_all_inv<<std::endl;
        std::cout<<"dq_all\n"<<dq_all<<std::endl;

      }

      for(int i=0;i<m_robot->numJoints();i++){
        m_robot->joint(i)->q += dq_all(BASE_DOF + i) * m_dt * 0.9;
      }

      m_robot->calcForwardKinematics();
//      vel_p *= vel_gain;
//      vel_r *= vel_gain;
//      v << dp, omega;
//      calcJacobian(J);
//      calcJacobianInverseNullspace(J, Jinv, Jnull);
//      dq = Jinv * v; // dq = pseudoInverse(J) * v
    }

    // Solve limb IK
//    bool solveLimbIK (IKparam& param, const std::string& limb_name, const double ratio_for_vel, const bool is_transition)
//    {
//        param.manip->calcInverseKinematics2Loop(param.target_p0, param.target_r0, 1.0, param.avoid_gain, param.reference_gain, &qrefv, ratio_for_vel,
//                                                param.localPos, param.localR);
//        checkIKTracking(param, limb_name, is_transition);
//        return true;
//    }
    // IK fail check
    void checkIKTracking (IKparam& param, const std::string& limb_name, const bool is_transition)
    {
        hrp::Vector3 vel_p, vel_r;
        vel_p = param.target_p0 - (param.target_link->p + param.target_link->R * param.localPos);
        rats::difference_rotation(vel_r, (param.target_link->R * param.localR), param.target_r0);
        if (vel_p.norm() > pos_ik_thre && is_transition) {
            if (param.pos_ik_error_count % ik_error_debug_print_freq == 0) {
                std::cerr << "[" << print_str << "] Too large IK error in " << limb_name << " (vel_p) = [" << vel_p(0) << " " << vel_p(1) << " " << vel_p(2) << "][m], count = " << param.pos_ik_error_count << std::endl;
            }
            param.pos_ik_error_count++;
            has_ik_failed = true;
        } else {
            param.pos_ik_error_count = 0;
        }
        if (vel_r.norm() > rot_ik_thre && is_transition) {
            if (param.rot_ik_error_count % ik_error_debug_print_freq == 0) {
                std::cerr << "[" << print_str << "] Too large IK error in " << limb_name << " (vel_r) = [" << vel_r(0) << " " << vel_r(1) << " " << vel_r(2) << "][rad], count = " << param.rot_ik_error_count << std::endl;
            }
            param.rot_ik_error_count++;
            has_ik_failed = true;
        } else {
            param.rot_ik_error_count = 0;
        }
    };
    // Reset IK fail params
    void resetIKFailParam() {
        has_ik_failed = false;
        for ( std::map<std::string, IKparam>::iterator it = ikp.begin(); it != ikp.end(); it++ ) {
            it->second.pos_ik_error_count = it->second.rot_ik_error_count = 0;
        }
    }
    // Get IKparam
//    void getIKParam (std::vector<std::string>& ee_vec /*, _CORBA_Unbounded_Sequence<OpenHRP::AutoBalancerService::IKLimbParameters>& ik_limb_parameters*/)
//    {
//        ik_limb_parameters.length(ee_vec.size());
//        for (size_t i = 0; i < ee_vec.size(); i++) {
//            IKparam& param = ikp[ee_vec[i]];
//            OpenHRP::AutoBalancerService::IKLimbParameters& ilp = ik_limb_parameters[i];
//            ilp.ik_optional_weight_vector.length(param.manip->numJoints());
//            std::vector<double> ov;
//            ov.resize(param.manip->numJoints());
//            param.manip->getOptionalWeightVector(ov);
//            for (size_t j = 0; j < param.manip->numJoints(); j++) {
//                ilp.ik_optional_weight_vector[j] = ov[j];
//            }
//            ilp.sr_gain = param.manip->getSRGain();
//            ilp.avoid_gain = param.avoid_gain;
//            ilp.reference_gain = param.reference_gain;
//            ilp.manipulability_limit = param.manip->getManipulabilityLimit();
//        }
//    };
//    // Set IKparam
//    void setIKParam (std::vector<std::string>& ee_vec, const _CORBA_Unbounded_Sequence<OpenHRP::AutoBalancerService::IKLimbParameters>& ik_limb_parameters)
//    {
//        std::cerr << "[" << print_str << "]  IK limb parameters" << std::endl;
//        bool is_ik_limb_parameter_valid_length = true;
//        if (ik_limb_parameters.length() != ee_vec.size()) {
//            is_ik_limb_parameter_valid_length = false;
//            std::cerr << "[" << print_str << "]   ik_limb_parameters invalid length! Cannot be set. (input = " << ik_limb_parameters.length() << ", desired = " << ee_vec.size() << ")" << std::endl;
//        } else {
//            for (size_t i = 0; i < ee_vec.size(); i++) {
//                if (ikp[ee_vec[i]].manip->numJoints() != ik_limb_parameters[i].ik_optional_weight_vector.length())
//                    is_ik_limb_parameter_valid_length = false;
//            }
//            if (is_ik_limb_parameter_valid_length) {
//                for (size_t i = 0; i < ee_vec.size(); i++) {
//                    IKparam& param = ikp[ee_vec[i]];
//                    const OpenHRP::AutoBalancerService::IKLimbParameters& ilp = ik_limb_parameters[i];
//                    std::vector<double> ov;
//                    ov.resize(param.manip->numJoints());
//                    for (size_t j = 0; j < param.manip->numJoints(); j++) {
//                        ov[j] = ilp.ik_optional_weight_vector[j];
//                    }
//                    param.manip->setOptionalWeightVector(ov);
//                    param.manip->setSRGain(ilp.sr_gain);
//                    param.avoid_gain = ilp.avoid_gain;
//                    param.reference_gain = ilp.reference_gain;
//                    param.manip->setManipulabilityLimit(ilp.manipulability_limit);
//                }
//            } else {
//                std::cerr << "[" << print_str << "]   ik_optional_weight_vector invalid length! Cannot be set. (input = [";
//                for (size_t i = 0; i < ee_vec.size(); i++) {
//                    std::cerr << ik_limb_parameters[i].ik_optional_weight_vector.length() << ", ";
//                }
//                std::cerr << "], desired = [";
//                for (size_t i = 0; i < ee_vec.size(); i++) {
//                    std::cerr << ikp[ee_vec[i]].manip->numJoints() << ", ";
//                }
//                std::cerr << "])" << std::endl;
//            }
//        }
//        if (is_ik_limb_parameter_valid_length) {
//            printIKparam(ee_vec);
//        }
//    };
    // Avoid limb stretch
    void limbStretchAvoidanceControl (const std::vector<hrp::Vector3>& target_p, const std::vector<std::string>& target_name)
    {
      m_robot->calcForwardKinematics();
      double tmp_d_root_height = 0.0, prev_d_root_height = d_root_height;
      if (use_limb_stretch_avoidance) {
        for (size_t i = 0; i < target_p.size(); i++) {
          // Check whether inside limb length limitation
          hrp::Link* parent_link = m_robot->link(ikp[target_name[i]].parent_name);
          hrp::Vector3 rel_target_p = target_p[i] - parent_link->p; // position from parent to target link (world frame)
          double limb_length_limitation = ikp[target_name[i]].max_limb_length - ikp[target_name[i]].limb_length_margin;
          double tmp = limb_length_limitation * limb_length_limitation - rel_target_p(0) * rel_target_p(0) - rel_target_p(1) * rel_target_p(1);
          if (rel_target_p.norm() > limb_length_limitation && tmp >= 0) {
            tmp_d_root_height = std::min(tmp_d_root_height, rel_target_p(2) + std::sqrt(tmp));
          }
        }
        // Change root link height depending on limb length
        d_root_height = tmp_d_root_height == 0.0 ? (- 1/limb_stretch_avoidance_time_const * d_root_height * m_dt + d_root_height) : tmp_d_root_height;
      } else {
        d_root_height = - 1/limb_stretch_avoidance_time_const * d_root_height * m_dt + d_root_height;
      }
      d_root_height = vlimit(d_root_height, prev_d_root_height + limb_stretch_avoidance_vlimit[0], prev_d_root_height + limb_stretch_avoidance_vlimit[1]);
      m_robot->rootLink()->p(2) += d_root_height;
    };
    double vlimit(const double value, const double llimit_value, const double ulimit_value)
    {
      if (value > ulimit_value) {
        return ulimit_value;
      } else if (value < llimit_value) {
        return llimit_value;
      }
      return value;
    };
    // Set parameter
    void printParam () const
    {
        std::cerr << "[" << print_str << "]   move_base_gain = " << move_base_gain << std::endl;
        std::cerr << "[" << print_str << "]   pos_ik_thre = " << pos_ik_thre << "[m], rot_ik_thre = " << rot_ik_thre << "[rad]" << std::endl;
    };
    // Set IKparam
    void printIKparam (std::vector<std::string>& ee_vec)
    {
      std::cerr << "[" << print_str << "]   ik_optional_weight_vectors = ";
      for (size_t i = 0; i < ee_vec.size(); i++) {
          IKparam& param = ikp[ee_vec[i]];
          std::vector<double> ov;
          ov.resize(param.manip->numJoints());
          param.manip->getOptionalWeightVector(ov);
          std::cerr << "[";
          for (size_t j = 0; j < param.manip->numJoints(); j++) {
              std::cerr << ov[j] << " ";
          }
          std::cerr << "]";
      }
      std::cerr << std::endl;
      std::cerr << "[" << print_str << "]   sr_gains = [";
      for (size_t i = 0; i < ee_vec.size(); i++) {
          std::cerr << ikp[ee_vec[i]].manip->getSRGain() << ", ";
      }
      std::cerr << "]" << std::endl;
      std::cerr << "[" << print_str << "]   avoid_gains = [";
      for (size_t i = 0; i < ee_vec.size(); i++) {
          std::cerr << ikp[ee_vec[i]].avoid_gain << ", ";
      }
      std::cerr << "]" << std::endl;
      std::cerr << "[" << print_str << "]   reference_gains = [";
      for (size_t i = 0; i < ee_vec.size(); i++) {
          std::cerr << ikp[ee_vec[i]].reference_gain << ", ";
      }
      std::cerr << "]" << std::endl;
      std::cerr << "[" << print_str << "]   manipulability_limits = [";
      for (size_t i = 0; i < ee_vec.size(); i++) {
          std::cerr << ikp[ee_vec[i]].manip->getManipulabilityLimit() << ", ";
      }
      std::cerr << "]" << std::endl;
    };
    hrp::Vector3 getEndEffectorPos(const std::string& limb_name){
      return ikp[limb_name].target_link->p + ikp[limb_name].target_link->R * ikp[limb_name].localPos;
    }
    hrp::Matrix33 getEndEffectorRot(const std::string& limb_name){
      return ikp[limb_name].target_link->R * ikp[limb_name].localR;
    }
};




















#endif //  FULLBODYIK_H
