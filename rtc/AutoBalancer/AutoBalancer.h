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

// </rtc-template>

// Service Consumer stub headers
// <rtc-template block="consumer_stub_h">

// </rtc-template>

using namespace RTC;

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
  bool setFootSteps(const OpenHRP::AutoBalancerService::FootstepSequence& fs);
  void waitFootSteps();
  bool startAutoBalancer(const ::OpenHRP::AutoBalancerService::StrSequence& limbs);
  bool stopAutoBalancer();
  bool setGaitGeneratorParam(const OpenHRP::AutoBalancerService::GaitGeneratorParam& i_param);
  bool getGaitGeneratorParam(OpenHRP::AutoBalancerService::GaitGeneratorParam& i_param);
  bool setAutoBalancerParam(const OpenHRP::AutoBalancerService::AutoBalancerParam& i_param);
  bool getAutoBalancerParam(OpenHRP::AutoBalancerService::AutoBalancerParam& i_param);
  bool getFootstepParam(OpenHRP::AutoBalancerService::FootstepParam& i_param);

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
  TimedAcceleration3D m_accRef;
  OutPort<TimedAcceleration3D> m_accRefOut;
  TimedBooleanSeq m_contactStates;
  OutPort<TimedBooleanSeq> m_contactStatesOut;
  TimedDoubleSeq m_controlSwingSupportTime;
  OutPort<TimedDoubleSeq> m_controlSwingSupportTimeOut;
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
    hrp::Vector3 target_p0, current_p0, target2foot_offset_pos;
    hrp::Matrix33 target_r0, current_r0, target2foot_offset_rot;
    rats::coordinates target_end_coords, current_end_coords;
    std::string target_name, base_name;
    hrp::JointPathExPtr manip;
    bool is_active;
    void getEndCoords(rats::coordinates& retc, const hrp::Vector3& _pos, const hrp::Matrix33& _rot)
    {
      retc.pos = _pos;
      retc.rot = _rot;
      retc.transform(rats::coordinates(target2foot_offset_pos, target2foot_offset_rot));
    };
    void getRobotEndCoords(rats::coordinates& retc, hrp::BodyPtr& _robot)
    {
      getEndCoords(retc, _robot->link(target_name)->p, _robot->link(target_name)->R);
    };
    void getTargetEndCoords(rats::coordinates& retc) { getEndCoords(retc, target_p0, target_r0); };
    void getCurrentEndCoords(rats::coordinates& retc) { getEndCoords(retc, current_p0, current_r0); };
  };
  void getCurrentParameters();
  void getTargetParameters();
  bool solveLimbIKforLimb (ABCIKparam& param);
  void solveLimbIK();
  void startABCparam(const ::OpenHRP::AutoBalancerService::StrSequence& limbs);
  void stopABCparam();
  void waitABCTransition();
  hrp::Matrix33 OrientRotationMatrix (const hrp::Matrix33& rot, const hrp::Vector3& axis1, const hrp::Vector3& axis2);
  void fixLegToCoords (const std::string& leg, const rats::coordinates& coords);
  void startWalking ();
  void stopWalking ();
  void copyRatscoords2Footstep(OpenHRP::AutoBalancerService::Footstep& out_fs, const rats::coordinates& in_fs);
  // static balance point offsetting
  void static_balance_point_proc_one(hrp::Vector3& tmp_input_sbp, const double ref_com_height);
  void calc_static_balance_point_from_forces(hrp::Vector3& sb_point, const hrp::Vector3& tmpcog, const double ref_com_height, std::vector<hrp::Vector3>& tmp_forces);

  // for gg
  typedef boost::shared_ptr<rats::gait_generator> ggPtr;
  ggPtr gg;
  bool gg_is_walking, gg_solved;
  // for abc
  hrp::Vector3 ref_cog, ref_zmp, prev_imu_sensor_pos, prev_imu_sensor_vel;
  enum {MODE_IDLE, MODE_ABC, MODE_SYNC_TO_IDLE, MODE_SYNC_TO_ABC} control_mode, return_control_mode;
  std::map<std::string, ABCIKparam> ikp;
  std::map<std::string, size_t> contact_states_index_map;
  hrp::dvector qorg, qrefv;
  hrp::Vector3 current_root_p, target_root_p;
  hrp::Matrix33 current_root_R, target_root_R;
  rats::coordinates fix_leg_coords;
  std::vector<hrp::Vector3> default_zmp_offsets;
  double m_dt, move_base_gain;
  hrp::BodyPtr m_robot;
  coil::Mutex m_mutex;

  double zmp_interpolate_time;
  interpolator *zmp_interpolator;
  interpolator *transition_interpolator;
  hrp::Vector3 input_zmp, input_basePos;
  hrp::Matrix33 input_baseRot;

  // static balance point offsetting
  hrp::Vector3 sbp_offset, sbp_cog_offset;
  enum {MODE_NO_FORCE, MODE_REF_FORCE} use_force;
  std::vector<hrp::Vector3> ref_forces;

  unsigned int m_debugLevel;
  bool is_legged_robot;
  int loop;
};


extern "C"
{
  void AutoBalancerInit(RTC::Manager* manager);
};

#endif // IMPEDANCE_H
