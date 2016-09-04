// -*- C++ -*-
/*!
 * @file  SoftErrorLimiter.h
 * @brief null component
 * @date  $Date$
 *
 * $Id$
 */

#ifndef SOFT_ERROR_LIMITER_H
#define SOFT_ERROR_LIMITER_H

#include <rtm/idl/BasicDataType.hh>
#include "hrpsys/idl/HRPDataTypes.hh"
#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>
#include <rtm/idl/BasicDataTypeSkel.h>
#include "JointLimitTable.h"

// Service implementation headers
// <rtc-template block="service_impl_h">
#include "SoftErrorLimiterService_impl.h"
#include "beep.h"

// </rtc-template>

// Service Consumer stub headers
// <rtc-template block="consumer_stub_h">

// </rtc-template>

using namespace RTC;

/**
   \brief sample RT component which has one data input port and one data output port
 */
class SoftErrorLimiter
  : public RTC::DataFlowComponentBase
{
 public:
  /**
     \brief Constructor
     \param manager pointer to the Manager
  */
  SoftErrorLimiter(RTC::Manager* manager);
  /**
     \brief Destructor
  */
  virtual ~SoftErrorLimiter();

  // The initialize action (on CREATED->ALIVE transition)
  // formaer rtc_init_entry()
  virtual RTC::ReturnCode_t onInitialize();

  // The finalize action (on ALIVE->END transition)
  // formaer rtc_exiting_entry()
  //virtual RTC::ReturnCode_t onFinalize();

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


 protected:
  // Configuration variable declaration
  // <rtc-template block="config_declare">
  
  // </rtc-template>

  TimedDoubleSeq m_qRef;
  TimedDoubleSeq m_qCurrent;
  OpenHRP::TimedLongSeqSeq m_servoState;
  TimedLongSeq m_beepCommand;

  // DataInPort declaration
  // <rtc-template block="inport_declare">
  InPort<TimedDoubleSeq> m_qRefIn;
  InPort<TimedDoubleSeq> m_qCurrentIn;
  InPort<OpenHRP::TimedLongSeqSeq> m_servoStateIn;
  
  // </rtc-template>

  // DataOutPort declaration
  // <rtc-template block="outport_declare">
  OutPort<TimedDoubleSeq> m_qOut;
  OutPort<OpenHRP::TimedLongSeqSeq> m_servoStateOut;
  OutPort<TimedLongSeq> m_beepCommandOut;
  
  // </rtc-template>

  // CORBA Port declaration
  // <rtc-template block="corbaport_declare">
  RTC::CorbaPort m_SoftErrorLimiterServicePort;
  
  // </rtc-template>

  // Service declaration
  // <rtc-template block="service_declare">
  SoftErrorLimiterService_impl m_service0;
  
  // </rtc-template>

  // Consumer declaration
  // <rtc-template block="consumer_declare">
  
  // </rtc-template>

 private:
  boost::shared_ptr<robot> m_robot;
  std::map<std::string, hrp::JointLimitTable> joint_limit_tables;
  unsigned int m_debugLevel;
  int dummy, position_limit_error_beep_freq, soft_limit_error_beep_freq, debug_print_freq;
  double dt;
  BeepClient bc;
  // Since this RTC is stable RTC, we support both direct beeping from this RTC and beepring through BeeperRTC.
  // If m_beepCommand is connected to BeeperRTC, is_beep_port_connected is true.
  bool is_beep_port_connected;
};


extern "C"
{
  void SoftErrorLimiterInit(RTC::Manager* manager);
};

#endif // SOFT_ERROR_LIMITER_H
