// -*- C++ -*-
/*!
 * @file  OccupancyGridMap3D.h
 * @brief null component
 * @date  $Date$
 *
 * $Id$
 */

#ifndef NULL_COMPONENT_H
#define NULL_COMPONENT_H

#include <rtm/idl/BasicDataType.hh>
#include <rtm/idl/InterfaceDataTypes.hh>
#include "hrpsys/idl/pointcloud.hh"
#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>
#include <rtm/idl/BasicDataTypeSkel.h>

namespace octomap{
    class OcTree;
};


// Service implementation headers
// <rtc-template block="service_impl_h">

// </rtc-template>

// Service Consumer stub headers
// <rtc-template block="consumer_stub_h">

// </rtc-template>

#include "OGMap3DService_impl.h"

using namespace RTC;

/**
   \brief sample RT component which has one data input port and one data output port
 */
class OccupancyGridMap3D
  : public RTC::DataFlowComponentBase
{
 public:
  /**
     \brief Constructor
     \param manager pointer to the Manager
  */
  OccupancyGridMap3D(RTC::Manager* manager);
  /**
     \brief Destructor
  */
  virtual ~OccupancyGridMap3D();

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

  OpenHRP::OGMap3D* getOGMap3D(const OpenHRP::AABB& region);
  void save(const char *filename);
  void clear();

 protected:
  // Configuration variable declaration
  // <rtc-template block="config_declare">
  
  // </rtc-template>

  PointCloudTypes::PointCloud m_cloud;
  TimedPose3D m_pose;
  TimedPoint3D m_sensorPos;
  TimedLong m_update;
  RangeData m_range;  

  // DataInPort declaration
  // <rtc-template block="inport_declare">
  InPort<RangeData> m_rangeIn;
  InPort<PointCloudTypes::PointCloud> m_cloudIn;
  InPort<TimedPose3D> m_poseIn;
  InPort<TimedPoint3D> m_sensorPosIn;
  InPort<TimedLong> m_updateIn;
  
  // </rtc-template>

  TimedLong m_updateSignal;

  // DataOutPort declaration
  // <rtc-template block="outport_declare">
  OutPort<TimedLong> m_updateOut;
  
  // </rtc-template>

  // CORBA Port declaration
  // <rtc-template block="corbaport_declare">
  RTC::CorbaPort m_OGMap3DServicePort;
  
  // </rtc-template>

  // Service declaration
  // <rtc-template block="service_declare">
  OGMap3DService_impl m_service0;
  
  // </rtc-template>

  // Consumer declaration
  // <rtc-template block="consumer_declare">
  
  // </rtc-template>

 private:
  octomap::OcTree *m_map, *m_knownMap;
  double m_occupiedThd, m_resolution;
  std::string m_initialMap;
  std::string m_knownMapPath;
  std::string m_cwd;
  coil::Mutex m_mutex;
  int m_debugLevel;
  int dummy;
};


extern "C"
{
  void OccupancyGridMap3DInit(RTC::Manager* manager);
};

#endif // NULL_COMPONENT_H
