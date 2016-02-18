// -*- mode: c++; -*-
/*!
 * @file  PDcontroller.h
 * @brief Sample PD component
 * @date  $Date$
 *
 * $Id$
 */

#ifndef PDcontroller_H
#define PDcontroller_H

#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/CorbaNaming.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>
#include <rtm/idl/BasicDataTypeSkel.h>

#include <hrpUtil/EigenTypes.h>
#include <hrpModel/ModelLoaderUtil.h>
#include <hrpModel/Body.h>
#include <hrpModel/Link.h>

// Service implementation headers
// <rtc-template block="service_impl_h">

// </rtc-template>

// Service Consumer stub headers
// <rtc-template block="consumer_stub_h">

// </rtc-template>

using namespace RTC;

class PDcontroller
  : public RTC::DataFlowComponentBase
{
 public:
  PDcontroller(RTC::Manager* manager);
  ~PDcontroller();

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

  // DataInPort declaration
  // <rtc-template block="inport_declare">
  TimedDoubleSeq m_angle;
  InPort<TimedDoubleSeq> m_angleIn;
  TimedDoubleSeq m_angleRef;
  InPort<TimedDoubleSeq> m_angleRefIn;
  
  // </rtc-template>

  // DataOutPort declaration
  // <rtc-template block="outport_declare">
  TimedDoubleSeq m_torque;
  OutPort<TimedDoubleSeq> m_torqueOut;
  
  // </rtc-template>

  // CORBA Port declaration
  // <rtc-template block="corbaport_declare">
  
  // </rtc-template>

  // Service declaration
  // <rtc-template block="service_declare">
  
  // </rtc-template>

  // Consumer declaration
  // <rtc-template block="consumer_declare">
  
  // </rtc-template>

 private:
  void readGainFile();
  hrp::BodyPtr m_robot;
  int dummy;
  double dt;     // sampling time of pd control
  double ref_dt; // sampling time of renference angles
  int step;      // current interpolation step
  int nstep;     // the number of steps to interpolate references
  std::ifstream gain;
  std::string gain_fname;
  hrp::dvector qold, qold_ref, Pgain, Dgain, tlimit_ratio;
  size_t dof, loop;
  unsigned int m_debugLevel;
};

extern "C"
{
   void PDcontrollerInit(RTC::Manager* manager);
};

#endif // PDcontroller_H
