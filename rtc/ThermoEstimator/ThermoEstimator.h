// -*- C++ -*-
/*!
 * @file  ThermoEstimator.h
 * @brief null component
 * @date  $Date$
 *
 * $Id$
 */

#ifndef TORQUE_FILTER_H
#define TORQUE_FILTER_H

#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>
#include <rtm/idl/BasicDataTypeSkel.h>

#include <hrpModel/Body.h>
#include <hrpModel/Link.h>
#include <hrpModel/JointPath.h>

// Service implementation headers
// <rtc-template block="service_impl_h">
// #include "ThermoEstimator_impl.h"

// </rtc-template>

// Service Consumer stub headers
// <rtc-template block="consumer_stub_h">

// </rtc-template>

using namespace RTC;

/**
   \brief sample RT component which has one data input port and one data output port
 */
class ThermoEstimator
  : public RTC::DataFlowComponentBase
{
 public:
  /**
     \brief Constructor
     \param manager pointer to the Manager
  */
  ThermoEstimator(RTC::Manager* manager);
  /**
     \brief Destructor
  */
  virtual ~ThermoEstimator();

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
  TimedDoubleSeq m_tauIn;
  TimedDoubleSeq m_tempOut;

  // DataInPort declaration
  // <rtc-template block="inport_declare">
  InPort<TimedDoubleSeq> m_tauInIn;
  
  // </rtc-template>

  // DataOutPort declaration
  // <rtc-template block="outport_declare">
  OutPort<TimedDoubleSeq> m_tempOutOut;
  
  // </rtc-template>

  // CORBA Port declaration
  // <rtc-template block="corbaport_declare">
  
  // </rtc-template>

  // Service declaration
  // <rtc-template block="service_declare">
  //RTC::CorbaPort m_ThermoEstimatorServicePort;
  
  // </rtc-template>

  // Consumer declaration
  // <rtc-template block="consumer_declare">
  //ThermoEstimatorService_impl m_ThermoEstimatorService;
  
  // </rtc-template>

 private:

  // Tnew = T + (P - ((T - Ta) / R) * dt) / C
  //      = T + ((Re*K^2/C) * tau^2) + ((1/RC) * (T - Ta) * dt)
  // * P = Re * I^2 = Re * (K * tau)^2
  struct MotorHeatParam {
    double tempreture; // current tempreture
    double currentCoeffs; // Re*K^2/C
    double thermoCoeffs; // 1/RC
    // default params for motor heat param
    MotorHeatParam(){
      tempreture = 30.0;
      currentCoeffs = 0.00003;
      thermoCoeffs = 0.001;
    }
  };
  
  double m_dt;
  unsigned int m_debugLevel;
  hrp::BodyPtr m_robot; // for numJoints
  double m_ambientTemp; // Ta
  std::vector<MotorHeatParam> m_motorHeatParams;
  
};


extern "C"
{
  void ThermoEstimatorInit(RTC::Manager* manager);
};

#endif // TORQUE_FILTER_H
