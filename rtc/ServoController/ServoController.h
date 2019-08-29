// -*- C++ -*-
/*!
 * @file  ServoController.h
 * @brief null component
 * @date  $Date$
 *
 * $Id$
 */

#ifndef SERVO_CONTROLLER_H
#define SERVO_CONTROLLER_H

#include <rtm/idl/BasicDataType.hh>
#include <rtm/idl/ExtendedDataTypes.hh>
#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>
#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/idl/ExtendedDataTypesSkel.h>

// Service implementation headers
// <rtc-template block="service_impl_h">
#include "ServoControllerService_impl.h"

// </rtc-template>

// Service Consumer stub headers
// <rtc-template block="consumer_stub_h">

// </rtc-template>

using namespace RTC;

class ServoSerial;

/**
   \brief sample RT component which has one data input port and one data output port
 */
class ServoController
  : public RTC::DataFlowComponentBase
{
 public:
  /**
     \brief Constructor
     \param manager pointer to the Manager
  */
  ServoController(RTC::Manager* manager);
  /**
     \brief Destructor
  */
  virtual ~ServoController();

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

  bool setJointAngle(short id, double angle, double tm);
  bool setJointAngles(const OpenHRP::ServoControllerService::dSequence angles, double tm);
  bool getJointAngle(short id, double &angle);
  bool getJointAngles(OpenHRP::ServoControllerService::dSequence_out &angles);
  bool addJointGroup(const char *gname, const ::OpenHRP::ServoControllerService::iSequence ids);
  bool removeJointGroup(const char *gname);
  bool setJointAnglesOfGroup(const char *gname, const ::OpenHRP::ServoControllerService::dSequence angles, double tm);
  bool setMaxTorque(short id, short percentage);
  bool setReset(short id);
  bool getDuration(short id, double &duration);
  bool getSpeed(short id, double &speed);
  bool getMaxTorque(short id, short &percentage);
  bool getTorque(short id, double &torque);
  bool getTemperature(short id, double &temperature);
  bool getVoltage(short id, double &voltage);
  bool servoOn();
  bool servoOff();

 protected:
  // Configuration variable declaration
  // <rtc-template block="config_declare">
  
  // </rtc-template>

  // DataInPort declaration
  // <rtc-template block="inport_declare">
  
  // </rtc-template>

  // DataOutPort declaration
  // <rtc-template block="outport_declare">
  
  // </rtc-template>

  // CORBA Port declaration
  // <rtc-template block="corbaport_declare">
  
  // </rtc-template>

  // Service declaration
  // <rtc-template block="service_declare">
  RTC::CorbaPort m_ServoControllerServicePort;
  
  // </rtc-template>

  // Consumer declaration
  // <rtc-template block="consumer_declare">
  ServoControllerService_impl m_service0;
  
  // </rtc-template>

 private:
  std::map<std::string, std::vector<int> > joint_groups;
  std::vector<int> servo_id;
  std::vector<double> servo_offset;
  std::vector<double> servo_dir;
  ServoSerial* serial;
};


extern "C"
{
  void ServoControllerInit(RTC::Manager* manager);
};

#endif // SERVO_CONTROLLER_H
