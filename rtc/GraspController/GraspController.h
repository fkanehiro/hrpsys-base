// -*- C++ -*-
/*!
 * @file  GraspController.h
 * @brief null component
 * @date  $Date$
 *
 * $Id$
 */

#ifndef GRASP_CONTROLLER_H
#define GRASP_CONTROLLER_H

#include <rtm/idl/BasicDataType.hh>
#include "hrpsys/idl/HRPDataTypes.hh"
#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>
#include <rtm/idl/BasicDataTypeSkel.h>

// Service implementation headers
// <rtc-template block="service_impl_h">
#include "GraspControllerService_impl.h"
#include <hrpModel/Body.h>

// </rtc-template>

// Service Consumer stub headers
// <rtc-template block="consumer_stub_h">

// </rtc-template>

using namespace RTC;

/**
   \brief sample RT component which has one data input port and one data output port
 */
class GraspController
  : public RTC::DataFlowComponentBase
{
 public:
  /**
     \brief Constructor
     \param manager pointer to the Manager
  */
  GraspController(RTC::Manager* manager);
  /**
     \brief Destructor
  */
  virtual ~GraspController();

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

  bool startGrasp(const char *name, double target_error);
  bool stopGrasp(const char *name);

 protected:
  // Configuration variable declaration
  // <rtc-template block="config_declare">
  
  // </rtc-template>

  TimedDoubleSeq m_qRef;
  TimedDoubleSeq m_qCurrent;
  TimedDoubleSeq m_q;

  // DataInPort declaration
  // <rtc-template block="inport_declare">
  InPort<TimedDoubleSeq> m_qRefIn;
  InPort<TimedDoubleSeq> m_qCurrentIn;
  InPort<TimedDoubleSeq> m_qIn;
  
  // </rtc-template>

  // DataOutPort declaration
  // <rtc-template block="outport_declare">
  OutPort<TimedDoubleSeq> m_qOut;
  
  // </rtc-template>

  // CORBA Port declaration
  // <rtc-template block="corbaport_declare">
  RTC::CorbaPort m_GraspControllerServicePort;
  
  // </rtc-template>

  // Service declaration
  // <rtc-template block="service_declare">
  GraspControllerService_impl m_service0;
  
  // </rtc-template>

  // Consumer declaration
  // <rtc-template block="consumer_declare">
  
  // </rtc-template>

 private:
  struct GraspJoint {
    int  id;
    double dir;
  };
  struct GraspParam {
    double time;
    double target_error;
    std::vector<GraspJoint> joints;
  };
  std::map<std::string, GraspParam > m_grasp_param;
  hrp::BodyPtr m_robot;
  double m_dt;
  unsigned int m_debugLevel;
  int dummy;
};


extern "C"
{
  void GraspControllerInit(RTC::Manager* manager);
};

#endif // SOFT_ERROR_LIMITER_H
