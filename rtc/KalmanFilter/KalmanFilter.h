// -*- C++ -*-
/*!
 * @file  KalmanFilter.h
 * @brief null component
 * @date  $Date$
 *
 * $Id$
 */

#ifndef NULL_COMPONENT_H
#define NULL_COMPONENT_H

#include <rtm/idl/BasicDataType.hh>
#include <rtm/idl/ExtendedDataTypes.hh>
#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>
#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/idl/ExtendedDataTypesSkel.h>
#include <hrpModel/Body.h>

#include "RPYKalmanFilter.h"
#include "EKFilter.h"

// Service implementation headers
// <rtc-template block="service_impl_h">
#include "KalmanFilterService_impl.h"

// </rtc-template>

// Service Consumer stub headers
// <rtc-template block="consumer_stub_h">

// </rtc-template>

using namespace RTC;

/**
   \brief sample RT component which has one data input port and one data output port
*/
class KalmanFilter
  : public RTC::DataFlowComponentBase
{
public:
  /**
     \brief Constructor
     \param manager pointer to the Manager
  */
  KalmanFilter(RTC::Manager* manager);
  /**
     \brief Destructor
  */
  virtual ~KalmanFilter();

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
  bool setKalmanFilterParam(const OpenHRP::KalmanFilterService::KalmanFilterParam& i_param);
  bool getKalmanFilterParam(OpenHRP::KalmanFilterService::KalmanFilterParam& i_param);
  bool resetKalmanFilterState();

protected:
  // Configuration variable declaration
  // <rtc-template block="config_declare">
  
  // </rtc-template>

  TimedAngularVelocity3D m_rate;
  TimedAcceleration3D m_acc;
  TimedAcceleration3D m_accRef;
  TimedOrientation3D m_rpy;
  TimedOrientation3D m_rpyRaw;
  TimedOrientation3D m_rpy_prev;
  TimedOrientation3D m_rpyRaw_prev;

  // DataInPort declaration
  // <rtc-template block="inport_declare">
  InPort<TimedAngularVelocity3D> m_rateIn;
  InPort<TimedAcceleration3D> m_accIn;
  InPort<TimedAcceleration3D> m_accRefIn;
  InPort<TimedAngularVelocity3D> m_rpyIn; // for dummy usage
  
  // </rtc-template>

  // DataOutPort declaration
  // <rtc-template block="outport_declare">
  OutPort<TimedOrientation3D> m_rpyOut;
  OutPort<TimedOrientation3D> m_rpyRawOut;
  RTC::TimedDoubleSeq m_qCurrent;
  RTC::InPort<RTC::TimedDoubleSeq> m_qCurrentIn;
  RTC::TimedOrientation3D m_baseRpyCurrent;
  RTC::OutPort<RTC::TimedOrientation3D> m_baseRpyCurrentOut;
  
  // </rtc-template>

  // CORBA Port declaration
  // <rtc-template block="corbaport_declare">
  
  // </rtc-template>

  // Service declaration
  // <rtc-template block="service_declare">
  RTC::CorbaPort m_KalmanFilterServicePort;
  
  // </rtc-template>

  // Consumer declaration
  // <rtc-template block="consumer_declare">
  KalmanFilterService_impl m_service0;
  
  // </rtc-template>

private:
  double m_dt;
  RPYKalmanFilter rpy_kf;
  EKFilter ekf_filter;
  hrp::BodyPtr m_robot;
  hrp::Matrix33 m_sensorR, sensorR_offset;
  hrp::Vector3 acc_offset;
  unsigned int m_debugLevel;
  int dummy, loop;
  OpenHRP::KalmanFilterService::KFAlgorithm kf_algorithm;
};


extern "C"
{
  void KalmanFilterInit(RTC::Manager* manager);
};

#endif // NULL_COMPONENT_H
