// -*- C++ -*-
/*!
 * @file  OpenNIGrabber.h
 * @brief Moving Least Squares Filter
 * @date  $Date$
 *
 * $Id$
 */

#ifndef OPENNI_GRABBER_H
#define OPENNI_GRABBER_H

#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>
#include <rtm/idl/BasicDataTypeSkel.h>
#include <pcl/io/openni2_grabber.h>
#include <pcl/io/pcd_io.h>
#include "hrpsys/idl/pointcloud.hh"
#include "hrpsys/idl/Img.hh"

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
class OpenNIGrabber
  : public RTC::DataFlowComponentBase
{
 public:
  /**
     \brief Constructor
     \param manager pointer to the Manager
  */
  OpenNIGrabber(RTC::Manager* manager);
  /**
     \brief Destructor
  */
  virtual ~OpenNIGrabber();

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

  Img::TimedCameraImage m_image;
  Img::TimedCameraImage m_depth;
  PointCloudTypes::PointCloud m_cloud;

  // DataInPort declaration
  // <rtc-template block="inport_declare">
  
  // </rtc-template>

  // DataOutPort declaration
  // <rtc-template block="outport_declare">
  OutPort<PointCloudTypes::PointCloud> m_cloudOut;
  OutPort<Img::TimedCameraImage> m_imageOut;
  OutPort<Img::TimedCameraImage> m_depthOut;
  
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
  void grabberCallbackColorImage(const boost::shared_ptr<pcl::io::Image>& image);
  void grabberCallbackDepthImage(const boost::shared_ptr<pcl::io::DepthImage>& image);
  void grabberCallbackColorAndDepthImage(const boost::shared_ptr<pcl::io::Image>& image, const boost::shared_ptr<pcl::io::DepthImage>& depth, float reciprocalFocalLength);
  void grabberCallbackPointCloudRGBA(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud);
  void grabberCallbackPointCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud);
  void outputColorImage(const boost::shared_ptr<pcl::io::Image>& image);
  void outputDepthImage(const boost::shared_ptr<pcl::io::DepthImage>& image);

  pcl::Grabber *m_interface;
  int m_debugLevel;
  bool m_outputColorImage;
  bool m_outputDepthImage;
  bool m_outputPointCloud;
  bool m_outputPointCloudRGBA;
  bool m_requestToWriteImage;
  bool m_requestToWritePointCloud;
  int dummy;
};


extern "C"
{
  void OpenNIGrabberInit(RTC::Manager* manager);
};

#endif // OPENNI_GRABBER_H
