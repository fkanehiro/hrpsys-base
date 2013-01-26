// -*- C++ -*-
/*!
 * @file  CollisionDetector.h
 * @brief collision detector component
 * @date  $Date$
 *
 * $Id$
 */

#ifndef COLLISION_DETECTOR_H
#define COLLISION_DETECTOR_H

#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>
#include <rtm/idl/BasicDataTypeSkel.h>
#include <hrpModel/Body.h>
#include <hrpModel/ColdetLinkPair.h>
#include <hrpModel/ModelLoaderUtil.h>
#include "GLscene.h"
#include "util/SDLUtil.h"
#include "util/LogManager.h"
#include "TimedPosture.h"
#include "interpolator.h"

#include "VclipLinkPair.h"
#include "CollisionDetectorService_impl.h"

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
class CollisionDetector
  : public RTC::DataFlowComponentBase
{
 public:
  /**
     \brief Constructor
     \param manager pointer to the Manager
  */
  CollisionDetector(RTC::Manager* manager);
  /**
     \brief Destructor
  */
  virtual ~CollisionDetector();

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

  bool setTolerance(const char *i_link_pair_name, double i_tolerance);


 protected:
  // Configuration variable declaration
  // <rtc-template block="config_declare">
  
  // </rtc-template>

  // DataInPort declaration
  // <rtc-template block="inport_declare">
  TimedDoubleSeq m_qRef;
  InPort<TimedDoubleSeq> m_qRefIn;
  TimedDoubleSeq m_qCurrent;
  InPort<TimedDoubleSeq> m_qCurrentIn;
  
  // </rtc-template>

  // DataOutPort declaration
  // <rtc-template block="outport_declare">
  TimedDoubleSeq m_q;
  OutPort<TimedDoubleSeq> m_qOut;
  
  // </rtc-template>

  // CORBA Port declaration
  // <rtc-template block="corbaport_declare">
  RTC::CorbaPort m_CollisionDetectorServicePort;
  
  // </rtc-template>

  // Service declaration
  // <rtc-template block="service_declare">
  CollisionDetectorService_impl m_service0;
  
  // </rtc-template>

  // Consumer declaration
  // <rtc-template block="consumer_declare">

  
  // </rtc-template>
  void setupVClipModel(hrp::BodyPtr i_body);
  void setupVClipModel(hrp::Link *i_link);

 private:
  class CollisionLinkPair {
  public:
      CollisionLinkPair(VclipLinkPairPtr i_pair) : point0(hrp::Vector3(0,0,0)), point1(hrp::Vector3(0,0,0)), distance(0) {
          pair = i_pair;
      }
      VclipLinkPairPtr pair;
      hrp::Vector3 point0, point1;
      double distance;
  };
  CollisionDetectorComponent::GLscene m_scene;
  LogManager<TimedPosture> m_log; 
  SDLwindow m_window;
  GLbody *m_glbody;
  std::vector<Vclip::Polyhedron *> m_VclipLinks;
  bool m_use_viewer;
  hrp::BodyPtr m_robot;
  std::map<std::string, CollisionLinkPair *> m_pair;
  int m_loop_for_check, m_collision_loop;
  bool m_safe_posture;
  int m_recover_time;
  double m_dt;
  int dummy;
  //
  double *m_recover_jointdata, *m_lastsafe_jointdata;
  interpolator* m_interpolator;
  double i_dt;
  int default_recover_time;
  unsigned int m_debugLevel;
};


extern "C"
{
  void CollisionDetectorInit(RTC::Manager* manager);
};

#endif // COLLISION_DETECTOR_H
