// -*- C++ -*-
/*!
 * @file  SoftwareEmergency.h
 * @brief 
 * @date  $Date$
 *
 * $Id$
 */

#ifndef SOFTWARE_EMERGENCY_H
#define SOFTWARE_EMERGENCY_H

#include <rtm/idl/BasicDataType.hh>
#include <hrpsys/idl/HRPDataTypes.hh>
#include <rtm/idl/ExtendedDataTypes.hh>
#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>
extern "C"{
#include "usbio.h"
}

// Service implementation headers
// <rtc-template block="service_impl_h">
#include "SoftwareEmergencyService_impl.h"

// </rtc-template>

// Service Consumer stub headers
// <rtc-template block="consumer_stub_h">

// </rtc-template>

using namespace RTC;

/**
   \brief sample RT component which has one data input port and one data output port
 */
class SoftwareEmergency
  : public RTC::DataFlowComponentBase
{
 public:
  /**
     \brief Constructor
     \param manager pointer to the Manager
  */
  SoftwareEmergency(RTC::Manager* manager);
  /**
     \brief Destructor
  */
  virtual ~SoftwareEmergency();

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

  bool switchModeTo(::OpenHRP::SoftwareEmergencyService::mode m);
  ::OpenHRP::SoftwareEmergencyService::mode getMode() { return m_mode; }
  
  bool setButtonStatus(CORBA::Long id, CORBA::Boolean on);
  
 protected:
  // Configuration variable declaration
  // <rtc-template block="config_declare">
  
  // </rtc-template>

  TimedDoubleSeq m_q;
  TimedDoubleSeq m_qRef, m_qRefMod;
  OpenHRP::TimedLongSeqSeq m_servoState;
  TimedLong m_emergencySignal;
  TimedLong m_buttonSignal;
  
  // DataInPort declaration
  // <rtc-template block="inport_declare">
  InPort<TimedDoubleSeq> m_qIn;
  InPort<TimedDoubleSeq> m_qRefIn;
  InPort<OpenHRP::TimedLongSeqSeq> m_servoStateIn;
  // </rtc-template>

  // DataOutPort declaration
  // <rtc-template block="outport_declare">
  OutPort<TimedDoubleSeq> m_qRefOut;
  OutPort<TimedLong> m_emergencySignalOut;
  OutPort<TimedLong> m_buttonSignalOut;
  
  // </rtc-template>

  // CORBA Port declaration
  // <rtc-template block="corbaport_declare">
  RTC::CorbaPort m_SoftwareEmergencyServicePort;
  
  // </rtc-template>

  // Service declaration
  // <rtc-template block="service_declare">
  SoftwareEmergencyService_impl m_SoftwareEmergencyService;
  
  // </rtc-template>

  // Consumer declaration
  // <rtc-template block="consumer_declare">
  
  // </rtc-template>

 private:
  bool isServoOn();
  bool isServoOn(int i);
  void checkServoError();

  ::OpenHRP::SoftwareEmergencyService::mode m_mode;
  std::vector<bool> m_button;
  usbio_t m_hd;
  double m_servoErrorLimit;
  std::vector<double> m_servoErrorLimits;
  double m_gain;
  double m_dt;
  bool m_check;
  int dummy;
};


extern "C"
{
  void SoftwareEmergencyInit(RTC::Manager* manager);
};

#endif // SOFTWARE_EMERGENCY_H
