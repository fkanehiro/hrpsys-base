// -*- C++ -*-
/*!
 * @file  VirtualCamera.h
 * @brief virtual camera component
 * @date  $Date$
 *
 * $Id$
 */

#ifndef VIRTUAL_CAMERA_H
#define VIRTUAL_CAMERA_H

#include <rtm/idl/BasicDataType.hh>
#include <rtm/idl/InterfaceDataTypes.hh>
#include "hrpsys/idl/HRPDataTypes.hh"
#include "hrpsys/idl/Img.hh"
#include "hrpsys/idl/pointcloud.hh"
#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>
#include <rtm/idl/BasicDataTypeSkel.h>
//Open CV headder
#include <cv.h>
#include <highgui.h>
//
#include "hrpsys/util/LogManager.h"
#include "hrpsys/util/SDLUtil.h"
#include "GLscene.h"
class GLcamera;
class RTCGLbody;

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
class VirtualCamera
  : public RTC::DataFlowComponentBase
{
 public:
  /**
     \brief Constructor
     \param manager pointer to the Manager
  */
  VirtualCamera(RTC::Manager* manager);
  /**
     \brief Destructor
  */
  virtual ~VirtualCamera();

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

  OpenHRP::SceneState m_sceneState;
  RTC::TimedPoint3D m_basePos;
  RTC::TimedOrientation3D m_baseRpy;
  RTC::TimedDoubleSeq m_q;

  // DataInPort declaration
  // <rtc-template block="inport_declare">
  InPort<OpenHRP::SceneState> m_sceneStateIn;
  InPort<RTC::TimedPoint3D> m_basePosIn;
  InPort<RTC::TimedOrientation3D> m_baseRpyIn;
  InPort<RTC::TimedDoubleSeq> m_qIn;
  
  // </rtc-template>

  Img::TimedCameraImage m_image;
  RangeData m_range;  
  PointCloudTypes::PointCloud m_cloud;
  TimedPose3D m_poseSensor;

  // DataOutPort declaration
  // <rtc-template block="outport_declare">
  OutPort<Img::TimedCameraImage> m_imageOut;
  OutPort<RangeData> m_rangeOut;
  OutPort<PointCloudTypes::PointCloud> m_cloudOut;
  OutPort<TimedPose3D> m_poseSensorOut;
  
  // </rtc-template>

  // CORBA Port declaration
  // <rtc-template block="corbaport_declare">
  
  // </rtc-template>

  // Service declaration
  // <rtc-template block="service_declare">
  
  // </rtc-template>

  // Consumer declaration
  // <rtc-template block="consumer_declare">
  
  // </rtc-template>

 private:
  void setupRangeData();  
  void setupPointCloud();  
  GLscene m_scene;
  LogManager<OpenHRP::SceneState> m_log;
  SDLwindow m_window;
  GLcamera *m_camera;
  bool m_generateRange;
  bool m_generatePointCloud;
  int m_generatePointCloudStep;
  std::string m_pcFormat;
  bool m_generateMovie, m_isGeneratingMovie;
  int m_debugLevel;
  CvVideoWriter *m_videoWriter;
  IplImage *m_cvImage;
  std::string m_projectName;
  std::string m_cameraName;
  std::map<std::string, RTCGLbody *> m_bodies;
  int dummy;
};


extern "C"
{
  void VirtualCameraInit(RTC::Manager* manager);
};

#endif // VIRTUAL_CAMERA_H
