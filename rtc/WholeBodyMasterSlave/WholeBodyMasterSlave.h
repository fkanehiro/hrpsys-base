// -*- C++ -*-
/*!
 * @file  WholeBodyMasterSlave.h
 * @brief WholeBodyMasterSlave component
 * @date  $Date$
 *
 * $Id$
 */

#ifndef WholeBodyMasterSlave_H
#define WholeBodyMasterSlave_H

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
// Service implementation headers
// <rtc-template block="service_impl_h">
#include "WholeBodyMasterSlaveService_impl.h"
#include "interpolator.h"
#include "../TorqueFilter/IIRFilter.h"
#include "../AutoBalancer/HumanMasterSlave.h"
#include "../AutoBalancer/AutoBalancer.h"

// </rtc-template>

using namespace RTC;

//static std::ostream& operator<<(std::ostream& os, const struct RTC::Time &tm)
//{
//    int pre = os.precision();
//    os.setf(std::ios::fixed);
//    os << std::setprecision(6)
//       << (tm.sec + tm.nsec/1e9)
//       << std::setprecision(pre);
//    os.unsetf(std::ios::fixed);
//    return os;
//}


class WholeBodyMasterSlave
  : public RTC::DataFlowComponentBase
{
 public:
  WholeBodyMasterSlave(RTC::Manager* manager);
  virtual ~WholeBodyMasterSlave();
  virtual RTC::ReturnCode_t onInitialize();
  virtual RTC::ReturnCode_t onFinalize();
  virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);
  virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);
  virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);
  bool startCountDownForWholeBodyMasterSlave(const double sec);
  bool stopHumanSync();
  bool setWholeBodyMasterSlaveParam(const OpenHRP::WholeBodyMasterSlaveService::WholeBodyMasterSlaveParam& i_param);
  bool getWholeBodyMasterSlaveParam(OpenHRP::WholeBodyMasterSlaveService::WholeBodyMasterSlaveParam& i_param);

 protected:
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
  // for debug
  TimedPoint3D m_cog;
  //for human tracker
  TimedPose3D m_htcom;
  InPort<TimedPose3D> m_htcomIn;
  TimedPose3D m_htrf;
  InPort<TimedPose3D> m_htrfIn;
  TimedPose3D m_htlf;
  InPort<TimedPose3D> m_htlfIn;
  TimedPose3D m_htrh;
  InPort<TimedPose3D> m_htrhIn;
  TimedPose3D m_htlh;
  InPort<TimedPose3D> m_htlhIn;
  TimedPose3D m_hthead;
  InPort<TimedPose3D> m_htheadIn;
  TimedPoint3D m_htzmp;
  InPort<TimedPoint3D> m_htzmpIn;
  TimedPoint3D m_actzmp;
  InPort<TimedPoint3D> m_actzmpIn;
  TimedDoubleSeq m_htrfw;
  InPort<TimedDoubleSeq> m_htrfwIn;
  TimedDoubleSeq m_htlfw;
  InPort<TimedDoubleSeq> m_htlfwIn;
  //ishiguro dbg
  TimedPose3D m_htcom_dbg;
  OutPort<TimedPose3D> m_htcom_dbgOut;
  TimedPose3D m_htrf_dbg;
  OutPort<TimedPose3D> m_htrf_dbgOut;
  TimedPose3D m_htlf_dbg;
  OutPort<TimedPose3D> m_htlf_dbgOut;
  TimedPose3D m_htrh_dbg;
  OutPort<TimedPose3D> m_htrh_dbgOut;
  TimedPose3D m_htlh_dbg;
  OutPort<TimedPose3D> m_htlh_dbgOut;
  TimedPose3D m_hthead_dbg;
  OutPort<TimedPose3D> m_hthead_dbgOut;
  TimedPoint3D m_htzmp_dbg;
  OutPort<TimedPoint3D> m_htzmp_dbgOut;
  TimedDoubleSeq m_htrfw_dbg;
  OutPort<TimedDoubleSeq> m_htrfw_dbgOut;
  TimedDoubleSeq m_htlfw_dbg;
  OutPort<TimedDoubleSeq> m_htlfw_dbgOut;
  TimedPose3D m_rpcom_dbg;
  OutPort<TimedPose3D> m_rpcom_dbgOut;
  TimedPose3D m_rprf_dbg;
  OutPort<TimedPose3D> m_rprf_dbgOut;
  TimedPose3D m_rplf_dbg;
  OutPort<TimedPose3D> m_rplf_dbgOut;
  TimedPose3D m_rprh_dbg;
  OutPort<TimedPose3D> m_rprh_dbgOut;
  TimedPose3D m_rplh_dbg;
  OutPort<TimedPose3D> m_rplh_dbgOut;
  TimedPose3D m_rphead_dbg;
  OutPort<TimedPose3D> m_rphead_dbgOut;
  TimedPoint3D m_rpzmp_dbg;
  OutPort<TimedPoint3D> m_rpzmp_dbgOut;
  TimedPoint3D m_rpdcp_dbg;
  OutPort<TimedPoint3D> m_rpdcp_dbgOut;
  TimedPoint3D m_rpacp_dbg;
  OutPort<TimedPoint3D> m_rpacp_dbgOut;

  TimedDoubleSeq m_invdyn_dbg;
  OutPort<TimedDoubleSeq> m_invdyn_dbgOut;

  OutPort<TimedDoubleSeq> m_qOut;
  RTC::OutPort<RTC::TimedPoint3D> m_zmpOut;
  OutPort<TimedPoint3D> m_basePosOut;
  OutPort<TimedOrientation3D> m_baseRpyOut;
  TimedDoubleSeq m_baseTform;
  OutPort<TimedDoubleSeq> m_baseTformOut;
  TimedDoubleSeq m_optionalData2;
  OutPort<TimedDoubleSeq> m_optionalDataOut;
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
  
  RTC::CorbaPort m_WholeBodyMasterSlaveServicePort;

  WholeBodyMasterSlaveService_impl m_service0;

 private:
  struct ABCIKparam {
    hrp::Vector3 target_p0, localPos, adjust_interpolation_target_p0, adjust_interpolation_org_p0;
    hrp::Matrix33 target_r0, localR, adjust_interpolation_target_r0, adjust_interpolation_org_r0;
    hrp::Link* target_link;
    bool is_active, has_toe_joint;
  };
//  void getTargetParameters();
  void solveFullbodyIKStrictCOM(const HRPPose3D& com_ref, const HRPPose3D& rf_ref, const HRPPose3D& lf_ref, const HRPPose3D& rh_ref, const HRPPose3D& lh_ref, const hrp::Vector3& head_ref);
  void processWholeBodyMasterSlave();
//  void calcDynamicsFilterCompensation(const hrp::Vector3 zmp_lip, const hrp::Vector3 zmp_fullbody);
//  void solveFullbodyIK ();
//  void startABCparam(const ::OpenHRP::WholeBodyMasterSlaveService::StrSequence& limbs);
//  void stopABCparam();
//  void waitABCTransition();
//  // Functions to calculate parameters for ABC output.
//  // Output parameters are EE, limbCOPOffset, contactStates, controlSwingSupportTime, toeheelPhaseRatio
//  void getOutputParametersForWalking ();
//  void getOutputParametersForABC ();
//  void getOutputParametersForIDLE ();
//  void interpolateLegNamesAndZMPOffsets();
//  void calcFixCoordsForAdjustFootstep (rats::coordinates& tmp_fix_coords);
//  void rotateRefForcesForFixCoords (rats::coordinates& tmp_fix_coords);
//  void updateTargetCoordsForHandFixMode (rats::coordinates& tmp_fix_coords);
//  void calculateOutputRefForces ();
//  hrp::Vector3 calcFootMidPosUsingZMPWeightMap ();
//  void updateWalkingVelocityFromHandError (rats::coordinates& tmp_fix_coords);
//  void calcReferenceJointAnglesForIK ();
//  hrp::Matrix33 OrientRotationMatrix (const hrp::Matrix33& rot, const hrp::Vector3& axis1, const hrp::Vector3& axis2);
//  void fixLegToCoords (const hrp::Vector3& fix_pos, const hrp::Matrix33& fix_rot);
//  void fixLegToCoords2 (rats::coordinates& tmp_fix_coords);
//  bool startWalking ();
//  void stopWalking ();
//  void copyRatscoords2Footstep(OpenHRP::WholeBodyMasterSlaveService::Footstep& out_fs, const rats::coordinates& in_fs);
//  // static balance point offsetting
//  void static_balance_point_proc_one(hrp::Vector3& tmp_input_sbp, const double ref_com_height);
//  void calc_static_balance_point_from_forces(hrp::Vector3& sb_point, const hrp::Vector3& tmpcog, const double ref_com_height, std::vector<hrp::Vector3>& tmp_forces);
//  hrp::Vector3 calc_vel_from_hand_error (const rats::coordinates& tmp_fix_coords);
  bool isOptionalDataContact (const std::string& ee_name)
  {
      return (std::fabs(m_optionalData.data[contact_states_index_map[ee_name]]-1.0)<0.1)?true:false;
  };
//  bool calc_inital_support_legs(const double& y, std::vector<rats::coordinates>& initial_support_legs_coords, std::vector<rats::leg_type>& initial_support_legs, rats::coordinates& start_ref_coords);
//
//  // for gg
//  typedef boost::shared_ptr<rats::gait_generator> ggPtr;
//  ggPtr gg;
//  bool gg_is_walking, gg_solved;
//  // for abc
  typedef boost::shared_ptr<SimpleFullbodyInverseKinematicsSolver> fikPtr;
  fikPtr fik;
//  hrp::Vector3 ref_cog, ref_zmp, prev_imu_sensor_pos, prev_imu_sensor_vel, hand_fix_initial_offset;
//  enum {BIPED, TROT, PACE, CRAWL, GALLOP} gait_type;
//  enum {MODE_IDLE, MODE_ABC, MODE_SYNC_TO_IDLE, MODE_SYNC_TO_ABC} control_mode;
//  std::map<std::string, ABCIKparam> ikp;
  std::map<std::string, size_t> contact_states_index_map;
//  std::map<std::string, hrp::VirtualForceSensorParam> m_vfs;
//  std::vector<std::string> sensor_names, leg_names, ee_vec;
//  hrp::Vector3 target_root_p;
//  hrp::Matrix33 target_root_R;
//  rats::coordinates fix_leg_coords;
//  std::vector<hrp::Vector3> default_zmp_offsets;
  double m_dt;
  hrp::BodyPtr m_robot;
//  coil::Mutex m_mutex;
//
//  double transition_interpolator_ratio, transition_time, zmp_transition_time, adjust_footstep_transition_time, leg_names_interpolator_ratio;
//  interpolator *zmp_offset_interpolator;
//  interpolator *transition_interpolator;
//  interpolator *adjust_footstep_interpolator;
//  interpolator *leg_names_interpolator;
  hrp::Vector3 input_zmp, input_basePos;
  hrp::Matrix33 input_baseRot;

  hrp::Vector3 rel_ref_zmp; // ref zmp in base frame

//  // static balance point offsetting
//  hrp::Vector3 sbp_offset, sbp_cog_offset;
//  enum {MODE_NO_FORCE, MODE_REF_FORCE} use_force;
//  std::vector<hrp::Vector3> ref_forces;
//
  unsigned int m_debugLevel;
  bool is_legged_robot;// is_stop_mode, is_hand_fix_mode, is_hand_fix_initial;
  int loop;
//  bool graspless_manip_mode;
//  std::string graspless_manip_arm;
//  hrp::Vector3 graspless_manip_p_gain;
//  rats::coordinates graspless_manip_reference_trans_coords;

  hrp::InvDynStateBuffer idsb;
  std::vector<IIRFilter> invdyn_zmp_filters;

  hrp::InvDynStateBuffer idsb2;
  std::vector<IIRFilter> invdyn_zmp_filters2;


  hrp::Vector3 ref_zmp_invdyn2;

  //for HumanSynchronizer
  boost::shared_ptr<HumanSynchronizer> hsp;
};

extern "C"
{
  void WholeBodyMasterSlaveInit(RTC::Manager* manager);
};

#endif // WholeBodyMasterSlave_H
