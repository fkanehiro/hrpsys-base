// -*- C++ -*-
/*!
 * @file  ImpedanceController.h
 * @brief impedance control component
 * @date  $Date$
 *
 * $Id$
 */

#ifndef IMPEDANCE_H
#define IMPEDANCE_H

#include <boost/interprocess/sync/interprocess_semaphore.hpp>
#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>
#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/idl/ExtendedDataTypesSkel.h>
#include <hrpModel/Body.h>
#include "JointPathEx.h"
// Service implementation headers
// <rtc-template block="service_impl_h">
#include "ImpedanceControllerService_impl.h"

// </rtc-template>

// Service Consumer stub headers
// <rtc-template block="consumer_stub_h">

// </rtc-template>

using namespace RTC;

class ImpedanceController
  : public RTC::DataFlowComponentBase
{
 public:
  ImpedanceController(RTC::Manager* manager);
  virtual ~ImpedanceController();

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

  bool setImpedanceControllerParam(OpenHRP::ImpedanceControllerService::impedanceParam i_param_);
  bool deleteImpedanceController(std::string i_name_);

 protected:
  // Configuration variable declaration
  // <rtc-template block="config_declare">
  
  // </rtc-template>

  // DataInPort declaration
  // <rtc-template block="inport_declare">
  TimedDoubleSeq m_qCurrent;
  InPort<TimedDoubleSeq> m_qCurrentIn;
  TimedDoubleSeq m_qRef;
  InPort<TimedDoubleSeq> m_qRefIn;
  std::vector<TimedDoubleSeq> m_force;
  std::vector<InPort<TimedDoubleSeq> *> m_forceIn;
  
  // </rtc-template>

  // DataOutPort declaration
  // <rtc-template block="outport_declare">
  TimedDoubleSeq m_q;
  OutPort<TimedDoubleSeq> m_qOut;
  
  // </rtc-template>

  // CORBA Port declaration
  // <rtc-template block="corbaport_declare">
  RTC::CorbaPort m_ImpedanceControllerServicePort;

  // </rtc-template>

  // Service declaration
  // <rtc-template block="service_declare">
  ImpedanceControllerService_impl m_service0;

  // </rtc-template>

  // Consumer declaration
  // <rtc-template block="consumer_declare">
  
  // </rtc-template>

 private:
  struct ImpedanceParam{
    std::string base_name, target_name;
    hrp::Vector3 target_p0, target_p1, current_p0, current_p1, current_p2;
    hrp::Vector3 target_r0, target_r1, current_r0, current_r1, current_r2;
    hrp::Vector3 force_offset_p, force_offset_r;
    double M_p, D_p, K_p;
    double M_r, D_r, K_r;
    double sr_gain, avoid_gain, reference_gain, manipulability_limit;
    int transition_count;
    hrp::JointPathExPtr manip;
  };
  struct VirtualForceSensorParam {
    hrp::Vector3 p;
    hrp::Matrix33 R;
  };
  std::map<std::string, ImpedanceParam> m_impedance_param;
  std::map<std::string, VirtualForceSensorParam> m_sensors;
  double m_dt;
  hrp::BodyPtr m_robot;
  int dummy;
  coil::Mutex m_mutex;
};


extern "C"
{
  void ImpedanceControllerInit(RTC::Manager* manager);
};

#endif // IMPEDANCE_H
