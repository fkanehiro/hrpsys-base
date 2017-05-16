// -*- C++ -*-
/*!
 * @file  AutoBalancer.h
 * @brief autobalancer component
 * @date  $Date$
 *
 * $Id$
 */

#ifndef AUTOBALANCER_H
#define AUTOBALANCER_H

#include <rtm/idl/BasicDataType.hh>
#include <rtm/idl/ExtendedDataTypes.hh>
#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>
#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/idl/ExtendedDataTypesSkel.h>
#include <hrpModel/Body.h>
#include "../ImpedanceController/JointPathEx.h"
#include "../ImpedanceController/RatsMatrix.h"
#include "GaitGenerator.h"
// Service implementation headers
// <rtc-template block="service_impl_h">
#include "AutoBalancerService_impl.h"
#include "interpolator.h"
#include "../TorqueFilter/IIRFilter.h"

// </rtc-template>

// Service Consumer stub headers
// <rtc-template block="consumer_stub_h">

// </rtc-template>

using namespace RTC;

static std::ostream& operator<<(std::ostream& os, const struct RTC::Time &tm)
{
    int pre = os.precision();
    os.setf(std::ios::fixed);
    os << std::setprecision(6)
       << (tm.sec + tm.nsec/1e9)
       << std::setprecision(pre);
    os.unsetf(std::ios::fixed);
    return os;
}

// Class for Simple Fullbody Inverse Kinematics
//   Input : target root pos and rot, target COG, target joint angles, target EE coords
//   Output : joint angles
//   Algorithm : Limb IK + move base
class SimpleFullbodyInverseKinematicsSolver
{
private:
    // Robot model for IK
    hrp::BodyPtr m_robot;
    // Org (current) joint angles before IK
    hrp::dvector qorg;
    // IK fail checking
    int ik_error_debug_print_freq;
    std::string print_str;
    bool has_ik_failed;
public:
    // IK parameter for each limb
    struct IKparam {
        // IK target EE coords
        hrp::Vector3 target_p0;
        hrp::Matrix33 target_r0;
        // EE offset, EE link
        hrp::Vector3 localPos;
        hrp::Matrix33 localR;
        hrp::Link* target_link;
        // IK solver and parameter
        hrp::JointPathExPtr manip;
        double avoid_gain, reference_gain;
        // IK fail checking
        size_t pos_ik_error_count, rot_ik_error_count;
        // Solve ik or not
        bool is_ik_enable;
        // Name of parent link in limb
        std::string parent_name;
        // Limb length
        double max_limb_length, limb_length_margin;
        IKparam ()
            : avoid_gain(0.001), reference_gain(0.01),
              pos_ik_error_count(0), rot_ik_error_count(0),
              limb_length_margin(0.02)
        {
        };
    };
    std::map<std::string, IKparam> ikp;
    // Used for ref joint angles overwrite before IK
    std::vector<int> overwrite_ref_ja_index_vec;
    // IK targets and current?
    hrp::dvector qrefv;
    hrp::Vector3 target_root_p;
    hrp::Matrix33 target_root_R;
    hrp::Vector3 current_root_p;
    hrp::Matrix33 current_root_R;
    // IK params
    double move_base_gain, ratio_for_vel;
    // For IK fail checking
    double pos_ik_thre, rot_ik_thre;
    struct RTC::Time current_tm;
    // For limb stretch avoidance
    double d_root_height, limb_stretch_avoidance_time_const, limb_stretch_avoidance_vlimit[2], m_dt;
    bool use_limb_stretch_avoidance;

    SimpleFullbodyInverseKinematicsSolver (hrp::BodyPtr& _robot, const std::string& _print_str, const double _dt)
        : m_robot(_robot),
          ik_error_debug_print_freq(static_cast<int>(0.2/_dt)), // once per 0.2 [s]
          has_ik_failed(false),
          overwrite_ref_ja_index_vec(), ikp(),
          move_base_gain(0.8), ratio_for_vel(1.0),
          pos_ik_thre(0.5*1e-3), // [m]
          rot_ik_thre((1e-2)*M_PI/180.0), // [rad]
          print_str(_print_str), m_dt(_dt),
          use_limb_stretch_avoidance(false), limb_stretch_avoidance_time_const(1.5)
    {
        qorg.resize(m_robot->numJoints());
        qrefv.resize(m_robot->numJoints());
        limb_stretch_avoidance_vlimit[0] = -1000 * 1e-3 * _dt; // lower limit
        limb_stretch_avoidance_vlimit[1] = 50 * 1e-3 * _dt; // upper limit
    };
    ~SimpleFullbodyInverseKinematicsSolver () {};

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
        hrp::Vector3 dif_cog(ratio_for_vel*_dif_cog);
        dif_cog(2) = m_robot->rootLink()->p(2) - target_root_p(2);
        m_robot->rootLink()->p = m_robot->rootLink()->p + -1 * move_base_gain * dif_cog;
        m_robot->rootLink()->R = target_root_R;
        // Avoid limb stretch
        {
          std::vector<hrp::Vector3> tmp_p;
          std::vector<std::string> tmp_name;
          for ( std::map<std::string, IKparam>::iterator it = ikp.begin(); it != ikp.end(); it++ ) {
            if (it->first.find("leg") != std::string::npos) {
              tmp_p.push_back(it->second.target_p0);
              tmp_name.push_back(it->first);
            }
          }
          limbStretchAvoidanceControl(tmp_p, tmp_name);
        }
        // Overwrite by ref joint angle
        for (size_t i = 0; i < overwrite_ref_ja_index_vec.size(); i++) {
            m_robot->joint(overwrite_ref_ja_index_vec[i])->q = qrefv[overwrite_ref_ja_index_vec[i]];
        }
        m_robot->calcForwardKinematics();
        for ( std::map<std::string, IKparam>::iterator it = ikp.begin(); it != ikp.end(); it++ ) {
            if (it->second.is_ik_enable) solveLimbIK (it->second, it->first, ratio_for_vel, is_transition);
        }
    };
    // Solve limb IK
    bool solveLimbIK (IKparam& param, const std::string& limb_name, const double ratio_for_vel, const bool is_transition)
    {
        param.manip->calcInverseKinematics2Loop(param.target_p0, param.target_r0, 1.0, param.avoid_gain, param.reference_gain, &qrefv, ratio_for_vel,
                                                param.localPos, param.localR);
        checkIKTracking(param, limb_name, is_transition);
        return true;
    }
    // IK fail check
    void checkIKTracking (IKparam& param, const std::string& limb_name, const bool is_transition)
    {
        hrp::Vector3 vel_p, vel_r;
        vel_p = param.target_p0 - (param.target_link->p + param.target_link->R * param.localPos);
        rats::difference_rotation(vel_r, (param.target_link->R * param.localR), param.target_r0);
        if (vel_p.norm() > pos_ik_thre && is_transition) {
            if (param.pos_ik_error_count % ik_error_debug_print_freq == 0) {
                std::cerr << "[" << print_str << "] [" << current_tm
                          << "] Too large IK error in " << limb_name << " (vel_p) = [" << vel_p(0) << " " << vel_p(1) << " " << vel_p(2) << "][m], count = " << param.pos_ik_error_count << std::endl;
            }
            param.pos_ik_error_count++;
            has_ik_failed = true;
        } else {
            param.pos_ik_error_count = 0;
        }
        if (vel_r.norm() > rot_ik_thre && is_transition) {
            if (param.rot_ik_error_count % ik_error_debug_print_freq == 0) {
                std::cerr << "[" << print_str << "] [" << current_tm
                          << "] Too large IK error in " << limb_name << " (vel_r) = [" << vel_r(0) << " " << vel_r(1) << " " << vel_r(2) << "][rad], count = " << param.rot_ik_error_count << std::endl;
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
    void getIKParam (std::vector<std::string>& ee_vec, _CORBA_Unbounded_Sequence<OpenHRP::AutoBalancerService::IKLimbParameters>& ik_limb_parameters)
    {
        ik_limb_parameters.length(ee_vec.size());
        for (size_t i = 0; i < ee_vec.size(); i++) {
            IKparam& param = ikp[ee_vec[i]];
            OpenHRP::AutoBalancerService::IKLimbParameters& ilp = ik_limb_parameters[i];
            ilp.ik_optional_weight_vector.length(param.manip->numJoints());
            std::vector<double> ov;
            ov.resize(param.manip->numJoints());
            param.manip->getOptionalWeightVector(ov);
            for (size_t j = 0; j < param.manip->numJoints(); j++) {
                ilp.ik_optional_weight_vector[j] = ov[j];
            }
            ilp.sr_gain = param.manip->getSRGain();
            ilp.avoid_gain = param.avoid_gain;
            ilp.reference_gain = param.reference_gain;
            ilp.manipulability_limit = param.manip->getManipulabilityLimit();
        }
    };
    // Set IKparam
    void setIKParam (std::vector<std::string>& ee_vec, const _CORBA_Unbounded_Sequence<OpenHRP::AutoBalancerService::IKLimbParameters>& ik_limb_parameters)
    {
        std::cerr << "[" << print_str << "]  IK limb parameters" << std::endl;
        bool is_ik_limb_parameter_valid_length = true;
        if (ik_limb_parameters.length() != ee_vec.size()) {
            is_ik_limb_parameter_valid_length = false;
            std::cerr << "[" << print_str << "]   ik_limb_parameters invalid length! Cannot be set. (input = " << ik_limb_parameters.length() << ", desired = " << ee_vec.size() << ")" << std::endl;
        } else {
            for (size_t i = 0; i < ee_vec.size(); i++) {
                if (ikp[ee_vec[i]].manip->numJoints() != ik_limb_parameters[i].ik_optional_weight_vector.length())
                    is_ik_limb_parameter_valid_length = false;
            }
            if (is_ik_limb_parameter_valid_length) {
                for (size_t i = 0; i < ee_vec.size(); i++) {
                    IKparam& param = ikp[ee_vec[i]];
                    const OpenHRP::AutoBalancerService::IKLimbParameters& ilp = ik_limb_parameters[i];
                    std::vector<double> ov;
                    ov.resize(param.manip->numJoints());
                    for (size_t j = 0; j < param.manip->numJoints(); j++) {
                        ov[j] = ilp.ik_optional_weight_vector[j];
                    }
                    param.manip->setOptionalWeightVector(ov);
                    param.manip->setSRGain(ilp.sr_gain);
                    param.avoid_gain = ilp.avoid_gain;
                    param.reference_gain = ilp.reference_gain;
                    param.manip->setManipulabilityLimit(ilp.manipulability_limit);
                }
            } else {
                std::cerr << "[" << print_str << "]   ik_optional_weight_vector invalid length! Cannot be set. (input = [";
                for (size_t i = 0; i < ee_vec.size(); i++) {
                    std::cerr << ik_limb_parameters[i].ik_optional_weight_vector.length() << ", ";
                }
                std::cerr << "], desired = [";
                for (size_t i = 0; i < ee_vec.size(); i++) {
                    std::cerr << ikp[ee_vec[i]].manip->numJoints() << ", ";
                }
                std::cerr << "])" << std::endl;
            }
        }
        if (is_ik_limb_parameter_valid_length) {
            printIKparam(ee_vec);
        }
    };
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
};

class AutoBalancer
  : public RTC::DataFlowComponentBase
{
 public:
  AutoBalancer(RTC::Manager* manager);
  virtual ~AutoBalancer();

  // The initialize action (on CREATED->ALIVE transition)
  // formaer rtc_init_entry()
 virtual RTC::ReturnCode_t onInitialize();

  // The finalize action (on ALIVE->END transition)
  // formaer rtc_exiting_entry()
  virtual RTC::ReturnCode_t onFinalize();

  // The startup action when ExecutionContext startup
  // former rtc_starting_entry()
  // virtual RTC::ReturnCode_t onStartup(RTC::UniqueId ec_id);

  // The shutdown action when ExecutionContext stop
  // former rtc_stopping_entry()
  // virtual RTC::ReturnCode_t onShutdown(RTC::UniqueId ec_id);

  // The activated action (Active state entry action)
  // former rtc_active_entry()
  virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);

  // The deactivated action (Active state exit action)
  // former rtc_active_exit()
  virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);

  // The execution action that is invoked periodically
  // former rtc_active_do()
  virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

  // The aborting action when main logic error occurred.
  // former rtc_aborting_entry()
  // virtual RTC::ReturnCode_t onAborting(RTC::UniqueId ec_id);

  // The error action in ERROR state
  // former rtc_error_do()
  // virtual RTC::ReturnCode_t onError(RTC::UniqueId ec_id);

  // The reset action that is invoked resetting
  // This is same but different the former rtc_init_entry()
  // virtual RTC::ReturnCode_t onReset(RTC::UniqueId ec_id);

  // The state update action that is invoked after onExecute() action
  // no corresponding operation exists in OpenRTm-aist-0.2.0
  // virtual RTC::ReturnCode_t onStateUpdate(RTC::UniqueId ec_id);

  // The action that is invoked when execution context's rate is changed
  // no corresponding operation exists in OpenRTm-aist-0.2.0
  // virtual RTC::ReturnCode_t onRateChanged(RTC::UniqueId ec_id);
  bool goPos(const double& x, const double& y, const double& th);
  bool goVelocity(const double& vx, const double& vy, const double& vth);
  bool goStop();
  bool emergencyStop ();
  bool setFootSteps(const OpenHRP::AutoBalancerService::FootstepSequence& fs, CORBA::Long overwrite_fs_idx);
  bool setFootSteps(const OpenHRP::AutoBalancerService::FootstepsSequence& fss, CORBA::Long overwrite_fs_idx);
  bool setFootStepsWithParam(const OpenHRP::AutoBalancerService::FootstepSequence& fs, const OpenHRP::AutoBalancerService::StepParamSequence& sps, CORBA::Long overwrite_fs_idx);
  bool setFootStepsWithParam(const OpenHRP::AutoBalancerService::FootstepsSequence& fss, const OpenHRP::AutoBalancerService::StepParamsSequence& spss, CORBA::Long overwrite_fs_idx);
  void waitFootSteps();
  void waitFootStepsEarly(const double tm);
  bool startAutoBalancer(const ::OpenHRP::AutoBalancerService::StrSequence& limbs);
  bool stopAutoBalancer();
  bool setGaitGeneratorParam(const OpenHRP::AutoBalancerService::GaitGeneratorParam& i_param);
  bool getGaitGeneratorParam(OpenHRP::AutoBalancerService::GaitGeneratorParam& i_param);
  bool setAutoBalancerParam(const OpenHRP::AutoBalancerService::AutoBalancerParam& i_param);
  bool getAutoBalancerParam(OpenHRP::AutoBalancerService::AutoBalancerParam& i_param);
  bool getFootstepParam(OpenHRP::AutoBalancerService::FootstepParam& i_param);
  bool adjustFootSteps(const OpenHRP::AutoBalancerService::Footstep& rfootstep, const OpenHRP::AutoBalancerService::Footstep& lfootstep);
  bool getRemainingFootstepSequence(OpenHRP::AutoBalancerService::FootstepSequence_out o_footstep, CORBA::Long& o_current_fs_idx);
  bool getGoPosFootstepsSequence(const double& x, const double& y, const double& th, OpenHRP::AutoBalancerService::FootstepsSequence_out o_footstep);
  bool releaseEmergencyStop();

 protected:
  // Configuration variable declaration
  // <rtc-template block="config_declare">
  
  // </rtc-template>

  // DataInPort declaration
  // <rtc-template block="inport_declare">
  TimedDoubleSeq m_qRef;
  InPort<TimedDoubleSeq> m_qRefIn;
  TimedPoint3D m_basePos;
  InPort<TimedPoint3D> m_basePosIn;
  TimedOrientation3D m_baseRpy;
  InPort<TimedOrientation3D> m_baseRpyIn;
  TimedPoint3D m_zmp;
  InPort<TimedPoint3D> m_zmpIn;
  TimedDoubleSeq m_optionalData;
  InPort<TimedDoubleSeq> m_optionalDataIn;
  std::vector<TimedDoubleSeq> m_ref_force;
  std::vector<InPort<TimedDoubleSeq> *> m_ref_forceIn;
  TimedLong m_emergencySignal;
  InPort<TimedLong> m_emergencySignalIn;
  TimedPoint3D m_diffCP;
  InPort<TimedPoint3D> m_diffCPIn;
  TimedBooleanSeq m_actContactStates;
  InPort<TimedBooleanSeq> m_actContactStatesIn;
  // for debug
  TimedPoint3D m_cog;
  
  // </rtc-template>

  // DataOutPort declaration
  // <rtc-template block="outport_declare">
  OutPort<TimedDoubleSeq> m_qOut;
  RTC::OutPort<RTC::TimedPoint3D> m_zmpOut;
  OutPort<TimedPoint3D> m_basePosOut;
  OutPort<TimedOrientation3D> m_baseRpyOut;
  TimedDoubleSeq m_baseTform;
  OutPort<TimedDoubleSeq> m_baseTformOut;
  TimedPose3D m_basePose;
  OutPort<TimedPose3D> m_basePoseOut;
  TimedAcceleration3D m_accRef;
  OutPort<TimedAcceleration3D> m_accRefOut;
  TimedBooleanSeq m_contactStates;
  OutPort<TimedBooleanSeq> m_contactStatesOut;
  TimedDoubleSeq m_toeheelRatio;
  OutPort<TimedDoubleSeq> m_toeheelRatioOut;
  TimedDoubleSeq m_controlSwingSupportTime;
  OutPort<TimedDoubleSeq> m_controlSwingSupportTimeOut;
  TimedBoolean m_walkingStates;
  OutPort<TimedBoolean> m_walkingStatesOut;
  TimedPoint3D m_sbpCogOffset;
  OutPort<TimedPoint3D> m_sbpCogOffsetOut;
  std::vector<TimedDoubleSeq> m_force;
  std::vector<OutPort<TimedDoubleSeq> *> m_ref_forceOut;
  std::vector<TimedPoint3D> m_limbCOPOffset;
  std::vector<OutPort<TimedPoint3D> *> m_limbCOPOffsetOut;
  // for debug
  OutPort<TimedPoint3D> m_cogOut;
  
  // </rtc-template>

  // CORBA Port declaration
  // <rtc-template block="corbaport_declare">
  RTC::CorbaPort m_AutoBalancerServicePort;

  // </rtc-template>

  // Service declaration
  // <rtc-template block="service_declare">
  AutoBalancerService_impl m_service0;

  // </rtc-template>

  // Consumer declaration
  // <rtc-template block="consumer_declare">
  
  // </rtc-template>

 private:
  struct ABCIKparam {
    hrp::Vector3 target_p0, localPos, adjust_interpolation_target_p0, adjust_interpolation_org_p0;
    hrp::Matrix33 target_r0, localR, adjust_interpolation_target_r0, adjust_interpolation_org_r0;
    hrp::Link* target_link;
    bool is_active, has_toe_joint;
  };
  void getTargetParameters();
  void solveFullbodyIK ();
  void startABCparam(const ::OpenHRP::AutoBalancerService::StrSequence& limbs);
  void stopABCparam();
  void waitABCTransition();
  // Functions to calculate parameters for ABC output.
  // Output parameters are EE, limbCOPOffset, contactStates, controlSwingSupportTime, toeheelPhaseRatio
  void getOutputParametersForWalking ();
  void getOutputParametersForABC ();
  void getOutputParametersForIDLE ();
  void interpolateLegNamesAndZMPOffsets();
  void calcFixCoordsForAdjustFootstep (rats::coordinates& tmp_fix_coords);
  void rotateRefForcesForFixCoords (rats::coordinates& tmp_fix_coords);
  void updateTargetCoordsForHandFixMode (rats::coordinates& tmp_fix_coords);
  void calculateOutputRefForces ();
  hrp::Vector3 calcFootMidPosUsingZMPWeightMap ();
  void updateWalkingVelocityFromHandError (rats::coordinates& tmp_fix_coords);
  void calcReferenceJointAnglesForIK ();
  hrp::Matrix33 OrientRotationMatrix (const hrp::Matrix33& rot, const hrp::Vector3& axis1, const hrp::Vector3& axis2);
  void fixLegToCoords (const hrp::Vector3& fix_pos, const hrp::Matrix33& fix_rot);
  void fixLegToCoords2 (rats::coordinates& tmp_fix_coords);
  bool startWalking ();
  void stopWalking ();
  void copyRatscoords2Footstep(OpenHRP::AutoBalancerService::Footstep& out_fs, const rats::coordinates& in_fs);
  // static balance point offsetting
  void static_balance_point_proc_one(hrp::Vector3& tmp_input_sbp, const double ref_com_height);
  void calc_static_balance_point_from_forces(hrp::Vector3& sb_point, const hrp::Vector3& tmpcog, const double ref_com_height, std::vector<hrp::Vector3>& tmp_forces);
  hrp::Vector3 calc_vel_from_hand_error (const rats::coordinates& tmp_fix_coords);
  bool isOptionalDataContact (const std::string& ee_name)
  {
      return (std::fabs(m_optionalData.data[contact_states_index_map[ee_name]]-1.0)<0.1)?true:false;
  };
  bool calc_inital_support_legs(const double& y, std::vector<rats::coordinates>& initial_support_legs_coords, std::vector<rats::leg_type>& initial_support_legs, rats::coordinates& start_ref_coords);

  // for gg
  typedef boost::shared_ptr<rats::gait_generator> ggPtr;
  ggPtr gg;
  bool gg_is_walking, gg_solved;
  // for abc
  typedef boost::shared_ptr<SimpleFullbodyInverseKinematicsSolver> fikPtr;
  fikPtr fik;
  hrp::Vector3 ref_cog, ref_zmp, prev_imu_sensor_pos, prev_imu_sensor_vel, hand_fix_initial_offset;
  enum {BIPED, TROT, PACE, CRAWL, GALLOP} gait_type;
  enum {MODE_IDLE, MODE_ABC, MODE_SYNC_TO_IDLE, MODE_SYNC_TO_ABC} control_mode;
  std::map<std::string, ABCIKparam> ikp;
  std::map<std::string, size_t> contact_states_index_map;
  std::map<std::string, hrp::VirtualForceSensorParam> m_vfs;
  std::vector<std::string> sensor_names, leg_names, ee_vec;
  hrp::Vector3 target_root_p;
  hrp::Matrix33 target_root_R;
  rats::coordinates fix_leg_coords;
  std::vector<hrp::Vector3> default_zmp_offsets;
  double m_dt;
  hrp::BodyPtr m_robot;
  coil::Mutex m_mutex;
  double d_pos_z_root, limb_stretch_avoidance_time_const, limb_stretch_avoidance_vlimit[2];
  bool use_limb_stretch_avoidance;

  double transition_interpolator_ratio, transition_time, zmp_transition_time, adjust_footstep_transition_time, leg_names_interpolator_ratio;
  interpolator *zmp_offset_interpolator;
  interpolator *transition_interpolator;
  interpolator *adjust_footstep_interpolator;
  interpolator *leg_names_interpolator;
  hrp::Vector3 input_zmp, input_basePos;
  hrp::Matrix33 input_baseRot;

  // static balance point offsetting
  hrp::Vector3 sbp_offset, sbp_cog_offset;
  enum {MODE_NO_FORCE, MODE_REF_FORCE} use_force;
  std::vector<hrp::Vector3> ref_forces;

  unsigned int m_debugLevel;
  bool is_legged_robot, is_stop_mode, is_hand_fix_mode, is_hand_fix_initial;
  int loop;
  bool graspless_manip_mode;
  std::string graspless_manip_arm;
  hrp::Vector3 graspless_manip_p_gain;
  rats::coordinates graspless_manip_reference_trans_coords;

  hrp::InvDynStateBuffer idsb;
  std::vector<IIRFilter> invdyn_zmp_filters;
};


extern "C"
{
  void AutoBalancerInit(RTC::Manager* manager);
};

#endif // IMPEDANCE_H
