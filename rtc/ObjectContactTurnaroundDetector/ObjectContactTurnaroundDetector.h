// -*- C++ -*-
/*!
 * @file  ObjectContactTurnaroundDetector.h
 * @brief object contact turnaround detector component
 * @date  $Date$
 *
 * $Id$
 */

#ifndef OBJECTCONTACTTURNAROUNDDETECTOR_H
#define OBJECTCONTACTTURNAROUNDDETECTOR_H

#include <rtm/idl/BasicDataType.hh>
#include <rtm/idl/ExtendedDataTypes.hh>
#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>
#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/idl/ExtendedDataTypesSkel.h>
#include <hrpUtil/Eigen3d.h>
#include <hrpModel/Body.h>
#include "ObjectContactTurnaroundDetectorBase.h"
// Service implementation headers
// <rtc-template block="service_impl_h">
#include "ObjectContactTurnaroundDetectorService_impl.h"

// </rtc-template>

// Service Consumer stub headers
// <rtc-template block="consumer_stub_h">

// </rtc-template>

using namespace RTC;

class ObjectContactTurnaroundDetector
  : public RTC::DataFlowComponentBase
{
 public:
  ObjectContactTurnaroundDetector(RTC::Manager* manager);
  virtual ~ObjectContactTurnaroundDetector();

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

  void startObjectContactTurnaroundDetection(const double i_ref_diff_wrench, const double i_max_time, const OpenHRP::ObjectContactTurnaroundDetectorService::StrSequence& i_ee_names);
  OpenHRP::ObjectContactTurnaroundDetectorService::DetectorMode checkObjectContactTurnaroundDetection();
  bool setObjectContactTurnaroundDetectorParam(const OpenHRP::ObjectContactTurnaroundDetectorService::objectContactTurnaroundDetectorParam &i_param_);
  bool getObjectContactTurnaroundDetectorParam(OpenHRP::ObjectContactTurnaroundDetectorService::objectContactTurnaroundDetectorParam& i_param_);
  bool getObjectForcesMoments(OpenHRP::ObjectContactTurnaroundDetectorService::Dbl3Sequence_out o_forces, OpenHRP::ObjectContactTurnaroundDetectorService::Dbl3Sequence_out o_moments, OpenHRP::ObjectContactTurnaroundDetectorService::DblSequence3_out o_3dofwrench);

 protected:
  // Configuration variable declaration
  // <rtc-template block="config_declare">
  
  // </rtc-template>

  // DataInPort declaration
  // <rtc-template block="inport_declare">
  TimedDoubleSeq m_qCurrent;
  InPort<TimedDoubleSeq> m_qCurrentIn;
  std::vector<TimedDoubleSeq> m_force;
  std::vector<InPort<TimedDoubleSeq> *> m_forceIn;
  TimedOrientation3D m_rpy;
  InPort<TimedOrientation3D> m_rpyIn;
  TimedBooleanSeq m_contactStates;
  InPort<TimedBooleanSeq> m_contactStatesIn;
  
  // </rtc-template>

  // DataOutPort declaration
  // <rtc-template block="outport_declare">
  TimedDoubleSeq m_otdData;
  OutPort<TimedDoubleSeq> m_otdDataOut;
  
  // </rtc-template>

  // CORBA Port declaration
  // <rtc-template block="corbaport_declare">
  RTC::CorbaPort m_ObjectContactTurnaroundDetectorServicePort;

  // </rtc-template>

  // Service declaration
  // <rtc-template block="service_declare">
  ObjectContactTurnaroundDetectorService_impl m_service0;

  // </rtc-template>

  // Consumer declaration
  // <rtc-template block="consumer_declare">
  
  // </rtc-template>

 private:

  struct ee_trans {
    std::string target_name, sensor_name;
    hrp::Vector3 localPos;
    hrp::Matrix33 localR;
    size_t index;
  };

  void updateRootLinkPosRot (TimedOrientation3D tmprpy);
  void calcFootMidCoords (hrp::Vector3& new_foot_mid_pos, hrp::Matrix33& new_foot_mid_rot);
  void calcFootOriginCoords (hrp::Vector3& foot_origin_pos, hrp::Matrix33& foot_origin_rot);
  void calcObjectContactTurnaroundDetectorState();

  std::map<std::string, ee_trans> ee_map;
  boost::shared_ptr<ObjectContactTurnaroundDetectorBase > otd;
  std::vector<std::string> otd_sensor_names;
  hrp::Vector3 otd_axis;
  double m_dt;
  hrp::BodyPtr m_robot;
  coil::Mutex m_mutex;
  unsigned int m_debugLevel;
  int dummy;
  int loop;
};


extern "C"
{
  void ObjectContactTurnaroundDetectorInit(RTC::Manager* manager);
};

#endif // OBJECTCONTACTTURNAROUNDDETECTOR_H
