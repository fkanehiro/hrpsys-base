// -*- C++ -*-
/*!
 * @file  Stabilizer.h
 * @brief stabilizer component
 * @date  $Date$
 *
 * $Id$
 */

#ifndef STABILIZER_COMPONENT_H
#define STABILIZER_COMPONENT_H

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

// Service implementation headers
// <rtc-template block="service_impl_h">
#include "StabilizerService_impl.h"
#include "TwoDofController.h"
#include "ZMPDistributor.h"
#include "../ImpedanceController/JointPathEx.h"
#include "../ImpedanceController/RatsMatrix.h"
#include "../TorqueFilter/IIRFilter.h"

// </rtc-template>

// Service Consumer stub headers
// <rtc-template block="consumer_stub_h">

// </rtc-template>

/**
   \brief sample RT component which has one data input port and one data output port
 */

class Stabilizer
  : public RTC::DataFlowComponentBase
{
 public:
  /**
     \brief Constructor
     \param manager pointer to the Manager
  */
  Stabilizer(RTC::Manager* manager);
  /**
     \brief Destructor
  */
  virtual ~Stabilizer();

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

  void startStabilizer(void);
  void stopStabilizer(void);
  void getCurrentParameters ();
  void getActualParameters ();
  void getTargetParameters ();
  void calcFootOriginCoords (hrp::Vector3& foot_origin_pos, hrp::Matrix33& foot_origin_rot);
  void sync_2_st ();
  void sync_2_idle();
  bool calcZMP(hrp::Vector3& ret_zmp, const double zmp_z);
  void calcStateForEmergencySignal();
  void calcRUNST();
  void moveBasePosRotForBodyRPYControl ();
  void calcSwingSupportLimbGain();
  void calcTPCC();
  void calcEEForceMomentControl();
  void calcSwingEEModification ();
  void limbStretchAvoidanceControl (const std::vector<hrp::Vector3>& ee_p, const std::vector<hrp::Matrix33>& ee_R);
  void getParameter(OpenHRP::StabilizerService::stParam& i_stp);
  void setParameter(const OpenHRP::StabilizerService::stParam& i_stp);
  void setBoolSequenceParam (std::vector<bool>& st_bool_values, const OpenHRP::StabilizerService::BoolSequence& output_bool_values, const std::string& prop_name);
  void setBoolSequenceParamWithCheckContact (std::vector<bool>& st_bool_values, const OpenHRP::StabilizerService::BoolSequence& output_bool_values, const std::string& prop_name);
  std::string getStabilizerAlgorithmString (OpenHRP::StabilizerService::STAlgorithm _st_algorithm);
  void waitSTTransition();
  // funcitons for calc final torque output
  void calcContactMatrix (hrp::dmatrix& tm, const std::vector<hrp::Vector3>& contact_p);
  void calcTorque ();
  void fixLegToCoords (const std::string& leg, const rats::coordinates& coords);
  void getFootmidCoords (rats::coordinates& ret);
  double calcDampingControl (const double tau_d, const double tau, const double prev_d,
                             const double DD, const double TT);
  hrp::Vector3 calcDampingControl (const hrp::Vector3& prev_d, const hrp::Vector3& TT);
  double calcDampingControl (const double prev_d, const double TT);
  hrp::Vector3 calcDampingControl (const hrp::Vector3& tau_d, const hrp::Vector3& tau, const hrp::Vector3& prev_d,
                                   const hrp::Vector3& DD, const hrp::Vector3& TT);
  double vlimit(double value, double llimit_value, double ulimit_value);
  hrp::Vector3 vlimit(const hrp::Vector3& value, double llimit_value, double ulimit_value);
  hrp::Vector3 vlimit(const hrp::Vector3& value, const hrp::Vector3& limit_value);
  hrp::Vector3 vlimit(const hrp::Vector3& value, const hrp::Vector3& llimit_value, const hrp::Vector3& ulimit_value);

  inline bool isContact (const size_t idx) // 0 = right, 1 = left
  {
    return (prev_act_force_z[idx] > 25.0);
  };
  inline int calcMaxTransitionCount ()
  {
      return (transition_time / dt);
  };
  void calcDiffFootOriginExtMoment ();

 protected:
  // Configuration variable declaration
  // <rtc-template block="config_declare">
  
  // </rtc-template>
  RTC::TimedDoubleSeq m_qCurrent;
  RTC::TimedDoubleSeq m_qRef;
  RTC::TimedDoubleSeq m_tau;
  RTC::TimedOrientation3D m_rpy;
  RTC::TimedPoint3D m_zmpRef;
  RTC::TimedPoint3D m_zmp;
  RTC::TimedPoint3D m_refCP;
  RTC::TimedPoint3D m_actCP;
  RTC::TimedPoint3D m_diffCP;
  RTC::TimedPoint3D m_diffFootOriginExtMoment;
  RTC::TimedPoint3D m_basePos;
  RTC::TimedOrientation3D m_baseRpy;
  RTC::TimedBooleanSeq m_contactStates;
  RTC::TimedDoubleSeq m_toeheelRatio;
  RTC::TimedDoubleSeq m_controlSwingSupportTime;
  std::vector<RTC::TimedPoint3D> m_limbCOPOffset;
  RTC::TimedBooleanSeq m_actContactStates;
  RTC::TimedDoubleSeq m_COPInfo;
  RTC::TimedLong m_emergencySignal;
  RTC::TimedDoubleSeq m_qRefSeq;
  RTC::TimedBoolean m_walkingStates;
  RTC::TimedPoint3D m_sbpCogOffset;
  // for debug ouput
  RTC::TimedPoint3D m_originRefZmp, m_originRefCog, m_originRefCogVel, m_originNewZmp;
  RTC::TimedPoint3D m_originActZmp, m_originActCog, m_originActCogVel;
  RTC::TimedOrientation3D m_actBaseRpy;
  RTC::TimedPoint3D m_currentBasePos;
  RTC::TimedOrientation3D m_currentBaseRpy;
  RTC::TimedDoubleSeq m_allRefWrench;
  RTC::TimedDoubleSeq m_allEEComp;
  RTC::TimedDoubleSeq m_debugData;
  
  // DataInPort declaration
  // <rtc-template block="inport_declare">
  RTC::InPort<RTC::TimedDoubleSeq> m_qCurrentIn;
  RTC::InPort<RTC::TimedDoubleSeq> m_qRefIn;
  RTC::InPort<RTC::TimedOrientation3D> m_rpyIn;
  RTC::InPort<RTC::TimedPoint3D> m_zmpRefIn;
  RTC::InPort<RTC::TimedPoint3D> m_basePosIn;
  RTC::InPort<RTC::TimedOrientation3D> m_baseRpyIn;
  RTC::InPort<RTC::TimedBooleanSeq> m_contactStatesIn;
  RTC::InPort<RTC::TimedDoubleSeq> m_toeheelRatioIn;
  RTC::InPort<RTC::TimedDoubleSeq> m_controlSwingSupportTimeIn;
  std::vector<RTC::InPort<RTC::TimedPoint3D> *> m_limbCOPOffsetIn;
  RTC::InPort<RTC::TimedDoubleSeq> m_qRefSeqIn;
  RTC::InPort<RTC::TimedBoolean> m_walkingStatesIn;
  RTC::InPort<RTC::TimedPoint3D> m_sbpCogOffsetIn;

  std::vector<RTC::TimedDoubleSeq> m_wrenches;
  std::vector<RTC::InPort<RTC::TimedDoubleSeq> *> m_wrenchesIn;
  std::vector<RTC::TimedDoubleSeq> m_ref_wrenches;
  std::vector<RTC::InPort<RTC::TimedDoubleSeq> *> m_ref_wrenchesIn;
  
  // </rtc-template>

  // DataOutPort declaration
  // <rtc-template block="outport_declare">
  RTC::OutPort<RTC::TimedDoubleSeq> m_qRefOut;
  RTC::OutPort<RTC::TimedDoubleSeq> m_tauOut;
  RTC::OutPort<RTC::TimedPoint3D> m_zmpOut;
  RTC::OutPort<RTC::TimedPoint3D> m_refCPOut;
  RTC::OutPort<RTC::TimedPoint3D> m_actCPOut;
  RTC::OutPort<RTC::TimedPoint3D> m_diffCPOut;
  RTC::OutPort<RTC::TimedPoint3D> m_diffFootOriginExtMomentOut;
  RTC::OutPort<RTC::TimedBooleanSeq> m_actContactStatesOut;
  RTC::OutPort<RTC::TimedDoubleSeq> m_COPInfoOut;
  RTC::OutPort<RTC::TimedLong> m_emergencySignalOut;
  // for debug output
  RTC::OutPort<RTC::TimedPoint3D> m_originRefZmpOut, m_originRefCogOut, m_originRefCogVelOut, m_originNewZmpOut;
  RTC::OutPort<RTC::TimedPoint3D> m_originActZmpOut, m_originActCogOut, m_originActCogVelOut;
  RTC::OutPort<RTC::TimedOrientation3D> m_actBaseRpyOut;
  RTC::OutPort<RTC::TimedPoint3D> m_currentBasePosOut;
  RTC::OutPort<RTC::TimedOrientation3D> m_currentBaseRpyOut;
  RTC::OutPort<RTC::TimedDoubleSeq> m_allRefWrenchOut;
  RTC::OutPort<RTC::TimedDoubleSeq> m_allEECompOut;
  RTC::OutPort<RTC::TimedDoubleSeq> m_debugDataOut;
  
  // </rtc-template>

  // CORBA Port declaration
  // <rtc-template block="corbaport_declare">
  
  // </rtc-template>

  // Service declaration
  // <rtc-template block="service_declare">
  RTC::CorbaPort m_StabilizerServicePort;
  
  // </rtc-template>

  // Consumer declaration
  // <rtc-template block="consumer_declare">
  StabilizerService_impl m_service0;
  
  // </rtc-template>

 private:
  // Stabilizer Parameters
  struct STIKParam {
    std::string target_name; // Name of end link
    std::string ee_name; // Name of ee (e.g., rleg, lleg, ...)
    std::string sensor_name; // Name of force sensor in the limb
    std::string parent_name; // Name of parent ling in the limb
    hrp::Vector3 localp; // Position of ee in end link frame (^{l}p_e = R_l^T (p_e - p_l))
    hrp::Vector3 localCOPPos; // Position offset of reference COP in end link frame (^{l}p_{cop} = R_l^T (p_{cop} - p_l) - ^{l}p_e)
    hrp::Matrix33 localR; // Rotation of ee in end link frame (^{l}R_e = R_l^T R_e)
    // For eefm
    hrp::Vector3 d_foot_pos, d_foot_rpy, ee_d_foot_rpy;
    hrp::Vector3 eefm_pos_damping_gain, eefm_pos_time_const_support, eefm_rot_damping_gain, eefm_rot_time_const, eefm_swing_rot_spring_gain, eefm_swing_pos_spring_gain, eefm_swing_rot_time_const, eefm_swing_pos_time_const, eefm_ee_moment_limit;
    double eefm_pos_compensation_limit, eefm_rot_compensation_limit;
    hrp::Vector3 ref_force, ref_moment;
    hrp::dvector6 eefm_ee_forcemoment_distribution_weight;
    double swing_support_gain, support_time;
    // For swing ee modification
    boost::shared_ptr<FirstOrderLowPassFilter<hrp::Vector3> > target_ee_diff_p_filter, target_ee_diff_r_filter;
    hrp::Vector3 target_ee_diff_p, d_pos_swing, d_rpy_swing, prev_d_pos_swing, prev_d_rpy_swing;
    hrp::Matrix33 target_ee_diff_r;
    // IK parameter
    double avoid_gain, reference_gain, max_limb_length, limb_length_margin;
    size_t ik_loop_count;
  };
  enum cmode {MODE_IDLE, MODE_AIR, MODE_ST, MODE_SYNC_TO_IDLE, MODE_SYNC_TO_AIR} control_mode;
  // members
  std::map<std::string, hrp::VirtualForceSensorParam> m_vfs;
  std::vector<hrp::JointPathExPtr> jpe_v;
  hrp::BodyPtr m_robot;
  coil::Mutex m_mutex;
  unsigned int m_debugLevel;
  hrp::dvector transition_joint_q, qorg, qrefv;
  std::vector<STIKParam> stikp;
  std::map<std::string, size_t> contact_states_index_map;
  std::vector<bool> ref_contact_states, prev_ref_contact_states, act_contact_states, is_ik_enable, is_feedback_control_enable, is_zmp_calc_enable;
  std::vector<double> toeheel_ratio;
  double dt;
  int transition_count, loop;
  int m_is_falling_counter;
  std::vector<int> m_will_fall_counter;
  int is_air_counter, detection_count_to_air;
  bool is_legged_robot, on_ground, is_emergency, is_seq_interpolating, reset_emergency_flag, eefm_use_force_difference_control, eefm_use_swing_damping, initial_cp_too_large_error, use_limb_stretch_avoidance;
  bool is_walking, is_estop_while_walking;
  hrp::Vector3 current_root_p, target_root_p;
  hrp::Matrix33 current_root_R, target_root_R, prev_act_foot_origin_rot, prev_ref_foot_origin_rot, target_foot_origin_rot, ref_foot_origin_rot;
  std::vector <hrp::Vector3> target_ee_p, rel_ee_pos, act_ee_p, projected_normal, act_force, ref_force, ref_moment;
  std::vector <hrp::Matrix33> target_ee_R, rel_ee_rot, act_ee_R;
  std::vector<std::string> rel_ee_name;
  rats::coordinates target_foot_midcoords;
  hrp::Vector3 ref_zmp, ref_cog, ref_cp, ref_cogvel, rel_ref_cp, prev_ref_cog, prev_ref_zmp;
  hrp::Vector3 act_zmp, act_cog, act_cogvel, act_cp, rel_act_zmp, rel_act_cp, prev_act_cog, act_base_rpy, current_base_rpy, current_base_pos, sbp_cog_offset, cp_offset, diff_cp;
  hrp::Vector3 foot_origin_offset[2];
  std::vector<double> prev_act_force_z;
  double zmp_origin_off, transition_smooth_gain, d_pos_z_root, limb_stretch_avoidance_time_const, limb_stretch_avoidance_vlimit[2], root_rot_compensation_limit[2];
  boost::shared_ptr<FirstOrderLowPassFilter<hrp::Vector3> > act_cogvel_filter;
  OpenHRP::StabilizerService::STAlgorithm st_algorithm;
  SimpleZMPDistributor* szd;
  // TPCC
  double k_tpcc_p[2], k_tpcc_x[2], d_rpy[2], k_brot_p[2], k_brot_tc[2];
  // RUN ST
  TwoDofController m_tau_x[2], m_tau_y[2], m_f_z;
  hrp::Vector3 pdr;
  double m_torque_k[2], m_torque_d[2]; // 3D-LIP parameters (0: x, 1: y)
  double pangx_ref, pangy_ref, pangx, pangy;
  double k_run_b[2], d_run_b[2];
  double rdx, rdy, rx, ry;
  // EEFM ST
  double eefm_k1[2], eefm_k2[2], eefm_k3[2], eefm_zmp_delay_time_const[2], eefm_body_attitude_control_gain[2], eefm_body_attitude_control_time_const[2];
  double eefm_pos_time_const_swing, eefm_pos_transition_time, eefm_pos_margin_time, eefm_gravitational_acceleration;
  std::vector<double> eefm_swing_damping_force_thre, eefm_swing_damping_moment_thre;
  hrp::Vector3 new_refzmp, rel_cog, ref_zmp_aux, diff_foot_origin_ext_moment;
  hrp::Vector3 pos_ctrl;
  hrp::Vector3 ref_total_force, ref_total_moment;
  hrp::Vector3 eefm_swing_pos_damping_gain, eefm_swing_rot_damping_gain;
  double total_mass, transition_time, cop_check_margin, contact_decision_threshold;
  std::vector<double> cp_check_margin, tilt_margin;
  OpenHRP::StabilizerService::EmergencyCheckMode emergency_check_mode;
};


extern "C"
{
  void StabilizerInit(RTC::Manager* manager);
};

#endif // STABILIZER_COMPONENT_H
