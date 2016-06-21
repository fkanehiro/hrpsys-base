// -*- C++ -*-
/*!
 * @file  RemoveForceSensorLinkOffset.h
 * @brief null component
 * @date  $Date$
 *
 * $Id$
 */

#ifndef REMOVEFORCESENSORLINKOFFSET_H
#define REMOVEFORCESENSORLINKOFFSET_H

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
#include <hrpModel/Link.h>
#include <hrpModel/JointPath.h>
#include <hrpUtil/EigenTypes.h>

#include "RemoveForceSensorLinkOffsetService_impl.h"
#include "../ImpedanceController/RatsMatrix.h"

// Service implementation headers
// <rtc-template block="service_impl_h">

// </rtc-template>

// Service Consumer stub headers
// <rtc-template block="consumer_stub_h">

// </rtc-template>

using namespace RTC;

/**
   \brief sample RT component which has one data input port and one data output port
 */
class RemoveForceSensorLinkOffset
  : public RTC::DataFlowComponentBase
{
 public:
  /**
     \brief Constructor
     \param manager pointer to the Manager
  */
  RemoveForceSensorLinkOffset(RTC::Manager* manager);
  /**
     \brief Destructor
  */
  virtual ~RemoveForceSensorLinkOffset();

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
  bool setForceMomentOffsetParam(const std::string& i_name_, const OpenHRP::RemoveForceSensorLinkOffsetService::forcemomentOffsetParam &i_param_);
  bool getForceMomentOffsetParam(const std::string& i_name_, OpenHRP::RemoveForceSensorLinkOffsetService::forcemomentOffsetParam& i_param_);
  bool loadForceMomentOffsetParams(const std::string& filename);
  bool dumpForceMomentOffsetParams(const std::string& filename);

 protected:
  // Configuration variable declaration
  // <rtc-template block="config_declare">
  
  // </rtc-template>
  // TimedDoubleSeq m_qRef;
  TimedDoubleSeq m_qCurrent;
  TimedOrientation3D m_rpy;
  
  // DataInPort declaration
  // <rtc-template block="inport_declare">
  InPort<TimedDoubleSeq> m_qCurrentIn;
  InPort<TimedOrientation3D> m_rpyIn;

  
  // </rtc-template>

  // DataOutPort declaration
  // <rtc-template block="outport_declare">
  std::vector<TimedDoubleSeq> m_force;
  std::vector<InPort<TimedDoubleSeq> *> m_forceIn;
  std::vector<OutPort<TimedDoubleSeq> *> m_forceOut;
  
  // </rtc-template>

  // DataOutPort declaration
  // <rtc-template block="outport_declare">
  
  // </rtc-template>

  // CORBA Port declaration
  // <rtc-template block="corbaport_declare">
  
  // </rtc-template>

  // Service declaration
  // <rtc-template block="service_declare">
  RTC::CorbaPort m_RemoveForceSensorLinkOffsetServicePort;
  
  // </rtc-template>

  // Consumer declaration
  // <rtc-template block="consumer_declare">
  RemoveForceSensorLinkOffsetService_impl m_service0;
  
  // </rtc-template>

 private:
  struct ForceMomentOffsetParam {
    hrp::Vector3 force_offset, moment_offset, link_offset_centroid;
    double link_offset_mass;

    ForceMomentOffsetParam ()
      : force_offset(hrp::Vector3::Zero()), moment_offset(hrp::Vector3::Zero()),
        link_offset_centroid(hrp::Vector3::Zero()), link_offset_mass(0)
    {};
  };
  void updateRootLinkPosRot (const hrp::Vector3& rpy);
  void printForceMomentOffsetParam(const std::string& i_name_);

  std::map<std::string, ForceMomentOffsetParam> m_forcemoment_offset_param;
  static const double grav = 9.80665; /* [m/s^2] */
  double m_dt;
  hrp::BodyPtr m_robot;
  unsigned int m_debugLevel;
};


extern "C"
{
  void RemoveForceSensorLinkOffsetInit(RTC::Manager* manager);
};

#endif // REMOVEFORCESENSORLINKOFFSET_H
