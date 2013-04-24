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

#include <boost/interprocess/sync/interprocess_semaphore.hpp>
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
  // virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);

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
  bool startABC(const OpenHRP::AutoBalancerService::AutoBalancerLimbParamSequence& alp);
  bool stopABC();
  bool setGaitGeneratorParam(const OpenHRP::AutoBalancerService::GaitGeneratorParam& i_param);
  bool getGaitGeneratorParam(OpenHRP::AutoBalancerService::GaitGeneratorParam& i_param);
  bool setAutoBalancerParam(const OpenHRP::AutoBalancerService::AutoBalancerParam& i_param);
  bool getAutoBalancerParam(OpenHRP::AutoBalancerService::AutoBalancerParam& i_param);

 protected:
  // Configuration variable declaration
  // <rtc-template block="config_declare">
  
  // </rtc-template>

  // DataInPort declaration
  // <rtc-template block="inport_declare">
  TimedDoubleSeq m_qRef;
  InPort<TimedDoubleSeq> m_qRefIn;
  
  // </rtc-template>

  // DataOutPort declaration
  // <rtc-template block="outport_declare">
  TimedDoubleSeq m_q;
  OutPort<TimedDoubleSeq> m_qOut;
  
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
    hrp::Vector3 target_p0, current_p0;
    hrp::Matrix33 target_r0, current_r0;
    std::string target_name, base_name;
    hrp::JointPathExPtr manip;
  };
  void robotstateOrg2qRef();
  bool solveLimbIKforLimb (ABCIKparam& param, const double transition_smooth_gain);
  void solveLimbIK();
  void startABCparam(const ::OpenHRP::AutoBalancerService::AutoBalancerLimbParamSequence& alp);
  void stopABCparam();
  void fixLegToCoords (const std::string& leg, const rats::coordinates& coords);
  void startWalking ();
  void stopWalking ();

  // for gg
  typedef boost::shared_ptr<rats::gait_generator> ggPtr;
  ggPtr gg;
  bool gg_is_walking, gg_ending, gg_solved;
  // for abc
  hrp::Vector3 target_com;
  int transition_count; // negative value when initing and positive value when deleting
  enum {MODE_IDLE, MODE_ABC, MODE_SYNC} control_mode;
  std::map<std::string, ABCIKparam> ikp;
  hrp::dvector transition_joint_q;
  hrp::dvector qorg;
  hrp::Vector3 base_pos_org, target_base_pos;
  hrp::Matrix33 base_rot_org, target_base_rot;
  rats::coordinates fix_leg_coords;
  std::vector<hrp::Vector3> default_zmp_offsets;
  double m_dt, move_base_gain;
  hrp::BodyPtr m_robot;
  coil::Mutex m_mutex;
  unsigned int m_debugLevel;
  int dummy;
  int loop;
};


extern "C"
{
  void AutoBalancerInit(RTC::Manager* manager);
};

#endif // IMPEDANCE_H
