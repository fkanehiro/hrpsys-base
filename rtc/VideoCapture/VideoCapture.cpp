// -*- C++ -*-
/*!
 * @file  VideoCapture.cpp
 * @brief video capture component
 * $Date$
 *
 * $Id$
 */

#include "VideoCapture.h"

// Module specification
// <rtc-template block="module_spec">
static const char* videocapture_spec[] =
  {
    "implementation_id", "VideoCapture",
    "type_name",         "VideoCapture",
    "description",       "null component",
    "version",           "1.0",
    "vendor",            "AIST",
    "category",          "example",
    "activity_type",     "DataFlowComponent",
    "max_instance",      "10",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables

    ""
  };
// </rtc-template>

VideoCapture::VideoCapture(RTC::Manager* manager)
  : RTC::DataFlowComponentBase(manager),
    // <rtc-template block="initializer">
    m_MultiCameraImagesOut ("MultiCameraImages", m_MultiCameraImages)
    // </rtc-template>
{
}

VideoCapture::~VideoCapture()
{
}



RTC::ReturnCode_t VideoCapture::onInitialize()
{
  std::cout << m_profile.instance_name << ": onInitialize()" << std::endl;
  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  
  // </rtc-template>

  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers

  // Set OutPort buffer
  addOutPort ("MultiCameraImages", m_MultiCameraImagesOut);
  
  // Set service provider to Ports
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  
  // </rtc-template>

  std::vector<int> devIds;
  //coil::stringTo (devIds, prop["camera_dev_id"].c_str ());
  devIds.push_back(0); // tempolary device number -> 0
  m_MultiCameraImages.data.image_seq.length (devIds.size ());
  for (unsigned int i = 0; i < devIds.size (); i++)
    {
      std::cout << "** devId:" << devIds[i] << std::endl;
      v4l_capture *cam = new v4l_capture ();
      cam->init(devIds[i], false);
      m_cameras.push_back (cam);
      m_MultiCameraImages.data.image_seq[i].image.width = cam->getWidth ();
      m_MultiCameraImages.data.image_seq[i].image.height = cam->getHeight ();
      m_MultiCameraImages.data.image_seq[i].image.raw_data.length (cam->getWidth () * cam->getHeight () * 3);
    }

  return RTC::RTC_OK;
}



/*
RTC::ReturnCode_t VideoCapture::onFinalize()
{
  return RTC::RTC_OK;
}
*/

RTC::ReturnCode_t VideoCapture::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t VideoCapture::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

RTC::ReturnCode_t VideoCapture::onActivated(RTC::UniqueId ec_id)
{
  std::cout << m_profile.instance_name<< ": onActivated(" << ec_id << ")" << std::endl;
  return RTC::RTC_OK;
}

RTC::ReturnCode_t VideoCapture::onDeactivated(RTC::UniqueId ec_id)
{
  std::cout << m_profile.instance_name<< ": onDeactivated(" << ec_id << ")" << std::endl;
  return RTC::RTC_OK;
}

RTC::ReturnCode_t VideoCapture::onExecute(RTC::UniqueId ec_id)
{
  //std::cout << m_profile.instance_name<< ": onExecute(" << ec_id << ")" << std::endl;
  capture();
  m_MultiCameraImagesOut.write();
  return RTC::RTC_OK;
}

void VideoCapture::capture()
{
  m_MultiCameraImages.error_code = 0;
  for (unsigned int i = 0; i < m_cameras.size (); i++)
    {
      m_MultiCameraImages.data.image_seq[i].image.format = Img::CF_RGB;
      m_MultiCameraImages.data.camera_set_id = 0;
      uchar *imgFrom = m_cameras[i]->capture();
      memcpy (m_MultiCameraImages.data.image_seq[i].image.raw_data.get_buffer(), imgFrom,
              m_MultiCameraImages.data.image_seq[i].image.raw_data.length() * sizeof (uchar));
    }
}

/*
RTC::ReturnCode_t VideoCapture::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t VideoCapture::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t VideoCapture::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t VideoCapture::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t VideoCapture::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/



extern "C"
{

  void VideoCaptureInit(RTC::Manager* manager)
  {
    RTC::Properties profile(videocapture_spec);
    manager->registerFactory(profile,
                             RTC::Create<VideoCapture>,
                             RTC::Delete<VideoCapture>);
  }

};


