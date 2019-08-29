// -*- C++ -*-
/*!
 * @file  TorqueController.h
 * @brief null component
 * @date  $Date$
 *
 * $Id$
 */

#ifndef TORQUE_CONTROLLER_H
#define TPRQUE_CONTROLLER_H

#include <rtm/idl/BasicDataType.hh>
#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>
#include <rtm/idl/BasicDataTypeSkel.h>

#include <hrpModel/Body.h>
#include <hrpModel/Link.h>
#include <hrpModel/JointPath.h>

#include "MotorTorqueController.h"

// Service implementation headers
// <rtc-template block="service_impl_h">

#include "TorqueControllerService_impl.h"

// </rtc-template>

// Service Consumer stub headers
// <rtc-template block="consumer_stub_h">

// </rtc-template>

using namespace RTC;

/**
   \brief sample RT component which has one data input port and one data output port
*/
class TorqueController
  : public RTC::DataFlowComponentBase
{
public:
/**
   \brief Constructor
   \param manager pointer to the Manager
*/
  TorqueController(RTC::Manager* manager);
/**
   \brief Destructor
*/
  virtual ~TorqueController();

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
  bool enableTorqueController(std::string jname);
  bool enableMultipleTorqueControllers(const OpenHRP::TorqueControllerService::StrSequence& jnames);
  bool disableTorqueController(std::string jname);
  bool disableMultipleTorqueControllers(const OpenHRP::TorqueControllerService::StrSequence& jnames);
  bool startTorqueControl(std::string jname);
  bool startMultipleTorqueControls(const OpenHRP::TorqueControllerService::StrSequence& jnames);
  bool stopTorqueControl(std::string jname);
  bool stopMultipleTorqueControls(const OpenHRP::TorqueControllerService::StrSequence& jnames);
  bool setReferenceTorque(std::string jname, double tauRef);
  bool setMultipleReferenceTorques(const OpenHRP::TorqueControllerService::StrSequence& jnames, const OpenHRP::TorqueControllerService::dSequence& tauRefs);
  bool setTorqueControllerParam(const std::string jname, const OpenHRP::TorqueControllerService::torqueControllerParam& i_param);
  bool getTorqueControllerParam(const std::string jname, OpenHRP::TorqueControllerService::torqueControllerParam& i_param);
  
protected:
// Configuration variable declaration
// <rtc-template block="config_declare">
  
// </rtc-template>
  TimedDoubleSeq m_tauCurrentIn;
  TimedDoubleSeq m_tauMaxIn;
  TimedDoubleSeq m_qCurrentIn;
  TimedDoubleSeq m_qRefIn;

  TimedDoubleSeq m_qRefOut;

// DataInPort declaration
// <rtc-template block="inport_declare">
  InPort<TimedDoubleSeq> m_tauCurrentInIn;
  InPort<TimedDoubleSeq> m_tauMaxInIn;
  InPort<TimedDoubleSeq> m_qCurrentInIn;
  InPort<TimedDoubleSeq> m_qRefInIn;
  
// </rtc-template>

// DataOutPort declaration
// <rtc-template block="outport_declare">
  OutPort<TimedDoubleSeq> m_qRefOutOut;
  
// </rtc-template>

// CORBA Port declaration
// <rtc-template block="corbaport_declare">
  
// </rtc-template>

// Service declaration
// <rtc-template block="service_declare">
  RTC::CorbaPort m_TorqueControllerServicePort;
  
// </rtc-template>

// Consumer declaration
// <rtc-template block="consumer_declare">
  TorqueControllerService_impl m_service0;
  
// </rtc-template>
  
private:
  double m_dt;
  unsigned int m_debugLevel;
  long long m_loop;
  hrp::BodyPtr m_robot;
  std::vector<MotorTorqueController> m_motorTorqueControllers;
  coil::Mutex m_mutex;
  void executeTorqueControl(hrp::dvector &dq);
  void updateParam(double &val, double &val_new);
  bool isDebug(int cycle = 20);
};


extern "C"
{
  void TorqueControllerInit(RTC::Manager* manager);
};

#endif // NULL_COMPONENT_H
