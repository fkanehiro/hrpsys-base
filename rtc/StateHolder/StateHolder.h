// -*- C++ -*-
/*!
 * @file  StateHolder.h
 * @brief state holder component
 * @date  $Date$
 *
 * $Id$
 */

#ifndef NULL_COMPONENT_H
#define NULL_COMPONENT_H

#include <rtm/idl/BasicDataType.hh>
#include <rtm/idl/ExtendedDataTypes.hh>
#include <semaphore.h>
#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>
#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/idl/ExtendedDataTypesSkel.h>

// Service implementation headers
// <rtc-template block="service_impl_h">
#include "StateHolderService_impl.h"
#include "TimeKeeperService_impl.h"
// </rtc-template>

// Service Consumer stub headers
// <rtc-template block="consumer_stub_h">

// </rtc-template>

using namespace RTC;

/**
   \brief RT component that do nothing and don't have ports. This component is used to create an execution context
 */
class StateHolder
  : public RTC::DataFlowComponentBase
{
 public:
  /**
     \brief Constructor
     \param manager pointer to the Manager
  */
  StateHolder(RTC::Manager* manager);
  /**
     \brief Destructor
  */
  virtual ~StateHolder();

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

  void goActual();
  void getCommand(StateHolderService::Command &com);  

  void wait(CORBA::Double tm);
 protected:
  // Configuration variable declaration
  // <rtc-template block="config_declare">
  
  // </rtc-template>
  TimedDoubleSeq m_currentQ;
  InPort<TimedDoubleSeq> m_currentQIn;
  InPort<TimedDoubleSeq> m_qIn;
  InPort<TimedDoubleSeq> m_tqIn;
  InPort<TimedPoint3D> m_basePosIn;
  InPort<TimedOrientation3D> m_baseRpyIn;
  InPort<TimedPoint3D> m_zmpIn;
  std::vector<InPort<TimedDoubleSeq> *> m_wrenchesIn;
  TimedDoubleSeq m_optionalData;
  InPort<TimedDoubleSeq> m_optionalDataIn;

  // DataInPort declaration
  // <rtc-template block="inport_declare">
  
  // </rtc-template>

  // DataOutPort declaration
  // <rtc-template block="outport_declare">
  TimedDoubleSeq m_q;
  TimedDoubleSeq m_tq;
  TimedPoint3D m_basePos;
  TimedOrientation3D m_baseRpy;
  TimedDoubleSeq m_baseTform;
  TimedPose3D m_basePose;
  TimedPoint3D m_zmp;
  std::vector<TimedDoubleSeq> m_wrenches;
  OutPort<TimedDoubleSeq> m_qOut;
  OutPort<TimedDoubleSeq> m_tqOut;
  OutPort<TimedPoint3D> m_basePosOut;
  OutPort<TimedOrientation3D> m_baseRpyOut;
  OutPort<TimedDoubleSeq> m_baseTformOut;
  OutPort<TimedPose3D> m_basePoseOut;
  OutPort<TimedPoint3D> m_zmpOut;
  std::vector<OutPort<TimedDoubleSeq> *> m_wrenchesOut;
  OutPort<TimedDoubleSeq> m_optionalDataOut;

  // </rtc-template>

  // CORBA Port declaration
  // <rtc-template block="corbaport_declare">
  RTC::CorbaPort m_StateHolderServicePort;
  RTC::CorbaPort m_TimeKeeperServicePort;

  // </rtc-template>

  // Service declaration
  // <rtc-template block="service_declare">
  StateHolderService_impl m_service0;
  TimeKeeperService_impl m_service1;  

  // </rtc-template>

  // Consumer declaration
  // <rtc-template block="consumer_declare">
  
  // </rtc-template>

 private:
  int m_timeCount;
  sem_t m_waitSem, m_timeSem;
  bool m_requestGoActual;
  double m_dt;
  int dummy;
};


extern "C"
{
  void StateHolderInit(RTC::Manager* manager);
};

#endif // NULL_COMPONENT_H
