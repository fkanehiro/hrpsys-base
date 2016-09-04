// -*- C++ -*-
/*!
 * @file  RobotHardware.h
 * @brief robot hardware component
 * @date  $Date$
 *
 * $Id$
 */

#ifndef ROBOT_HARDWARE_H
#define ROBOT_HARDWARE_H

#include <rtm/idl/BasicDataType.hh>
#include <rtm/idl/ExtendedDataTypes.hh>
#include "hrpsys/idl/HRPDataTypes.hh"
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
#include "RobotHardwareService_impl.h"

// </rtc-template>

// Service Consumer stub headers
// <rtc-template block="consumer_stub_h">

// </rtc-template>

using namespace RTC;

class robot;

/**
   \brief RT component that do nothing and don't have ports. This component is used to create an execution context
 */
class RobotHardware
  : public RTC::DataFlowComponentBase
{
 public:
  /**
     \brief Constructor
     \param manager pointer to the Manager
  */
  RobotHardware(RTC::Manager* manager);
  /**
     \brief Destructor
  */
  virtual ~RobotHardware();

  // The initialize action (on CREATED->ALIVE transition)
  // formaer rtc_init_entry()
  virtual RTC::ReturnCode_t onInitialize();

  // The finalize action (on ALIVE->END transition)
  // formaer rtc_exiting_entry()
  // virtual RTC::ReturnCode_t onFinalize();

  // The startup action when ExecutionContext startup
  // former rtc_starting_entry()
  // virtual RTC::ReturnCode_t onStartup(RTC::UniqueId ec_id);

  // The shutdown action when ExecutionContext stop
  // former rtc_stopping_entry()
  // virtual RTC::ReturnCode_t onShutdown(RTC::UniqueId ec_id);

  // The activated action (Active state entry action)
  // former rtc_active_entry()
  // virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);

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

  virtual inline void getTimeNow(Time &tm) {
      coil::TimeValue coiltm(coil::gettimeofday());
      tm.sec  = coiltm.sec();
      tm.nsec = coiltm.usec() * 1000;
  };

 protected:
  // Configuration variable declaration
  // <rtc-template block="config_declare">
  int m_isDemoMode;  
  
  // </rtc-template>

  // DataInPort declaration
  // <rtc-template block="inport_declare">

  /**
     \brief array of reference angles of joint with jointId
  */
  TimedDoubleSeq m_qRef;
  InPort<TimedDoubleSeq> m_qRefIn;
  /**
     \brief array of reference velocities of joint with jointId
  */
  TimedDoubleSeq m_dqRef;
  InPort<TimedDoubleSeq> m_dqRefIn;
  /**
     \brief array of reference torques of joint with jointId
  */
  TimedDoubleSeq m_tauRef;
  InPort<TimedDoubleSeq> m_tauRefIn;
  
  // </rtc-template>

  /**
     \brief array of actual angles of joint with jointId
  */
  TimedDoubleSeq m_q;
  /**
     \brief array of actual velocities of joint with jointId
  */
  TimedDoubleSeq m_dq;
  /**
     \brief array of actual torques of joint with jointId
  */
  TimedDoubleSeq m_tau;
  /**
     \brief array of commanded torques of joint with jointId
  */
  TimedDoubleSeq m_ctau;
  /**
     \brief vector of actual acceleration (vector length = number of acceleration sensors)
  */
  std::vector<TimedAcceleration3D> m_acc;
  /**
     \brief vector of actual angular velocity (vector length = number of rate sensors)
  */
  std::vector<TimedAngularVelocity3D> m_rate;
  /**
     \brief vector of actual 6D wrench (vector length = number of F/T sensors)
            6D wrench vector = 3D force + 3D moment = fx, fy, fz, nx, ny, nz
  */
  std::vector<TimedDoubleSeq> m_force;
  OpenHRP::TimedLongSeqSeq m_servoState;
  TimedLong m_emergencySignal;

  // DataOutPort declaration
  // <rtc-template block="outport_declare">
  OutPort<TimedDoubleSeq> m_qOut;
  OutPort<TimedDoubleSeq> m_dqOut;
  OutPort<TimedDoubleSeq> m_tauOut;
  OutPort<TimedDoubleSeq> m_ctauOut;
  std::vector<OutPort<TimedAcceleration3D> *> m_accOut;
  std::vector<OutPort<TimedAngularVelocity3D> *> m_rateOut;
  std::vector<OutPort<TimedDoubleSeq> *> m_forceOut;
  OutPort<OpenHRP::TimedLongSeqSeq> m_servoStateOut;
  OutPort<TimedLong> m_emergencySignalOut;

  // </rtc-template>

  // CORBA Port declaration
  // <rtc-template block="corbaport_declare">
  RTC::CorbaPort m_RobotHardwareServicePort;
  
  // </rtc-template>

  // Service declaration
  // <rtc-template block="service_declare">
  RobotHardwareService_impl m_service0;
  
  // </rtc-template>

  // Consumer declaration
  // <rtc-template block="consumer_declare">
  
  // </rtc-template>

  robot *robot_ptr(void) { return m_robot.get(); };
 private:
  int dummy;
  boost::shared_ptr<robot> m_robot;
};


extern "C"
{
  void RobotHardwareInit(RTC::Manager* manager);
};

#endif // ROBOT_HARDWARE_H
