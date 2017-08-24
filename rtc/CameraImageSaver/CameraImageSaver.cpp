// -*- C++ -*-
/*!
 * @file  CameraImageSaver.cpp
 * @brief camera image saver
 * $Date$
 *
 * $Id$
 */

#include <cv.h>
#include <highgui.h>
#include "CameraImageSaver.h"

// Module specification
// <rtc-template block="module_spec">
static const char* spec[] =
  {
    "implementation_id", "CameraImageSaver",
    "type_name",         "CameraImageSaver",
    "description",       "camera image saver",
    "version",           HRPSYS_PACKAGE_VERSION,
    "vendor",            "AIST",
    "category",          "example",
    "activity_type",     "DataFlowComponent",
    "max_instance",      "10",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    "conf.defalt.basename", "image",

    ""
  };
// </rtc-template>

CameraImageSaver::CameraImageSaver(RTC::Manager* manager)
  : RTC::DataFlowComponentBase(manager),
    // <rtc-template block="initializer">
    m_imageIn("image", m_image),
    // </rtc-template>
    m_count(0),
    dummy(0)
{
}

CameraImageSaver::~CameraImageSaver()
{
}



RTC::ReturnCode_t CameraImageSaver::onInitialize()
{
  //std::cout << m_profile.instance_name << ": onInitialize()" << std::endl;
  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  bindParameter("basename", m_basename, "image");
  
  // </rtc-template>

  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("image", m_imageIn);

  // Set OutPort buffer
  
  // Set service provider to Ports
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  
  // </rtc-template>

  RTC::Properties& prop = getProperties();

  return RTC::RTC_OK;
}



/*
RTC::ReturnCode_t CameraImageSaver::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t CameraImageSaver::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t CameraImageSaver::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

RTC::ReturnCode_t CameraImageSaver::onActivated(RTC::UniqueId ec_id)
{
  std::cout << m_profile.instance_name<< ": onActivated(" << ec_id << ")" << std::endl;
  return RTC::RTC_OK;
}

RTC::ReturnCode_t CameraImageSaver::onDeactivated(RTC::UniqueId ec_id)
{
  std::cout << m_profile.instance_name<< ": onDeactivated(" << ec_id << ")" << std::endl;
  return RTC::RTC_OK;
}

RTC::ReturnCode_t CameraImageSaver::onExecute(RTC::UniqueId ec_id)
{
  //std::cout << m_profile.instance_name<< ": onExecute(" << ec_id << ")" << std::endl;
  if (m_imageIn.isNew()){
    m_imageIn.read();

    IplImage* cvImage;
    switch (m_image.data.image.format){
    case Img::CF_RGB:
      cvImage = cvCreateImage(cvSize(m_image.data.image.width,
                                     m_image.data.image.height),
                              IPL_DEPTH_8U, 3);
      break;
    case Img::CF_GRAY:
      cvImage = cvCreateImage(cvSize(m_image.data.image.width,
                                     m_image.data.image.height),
                              IPL_DEPTH_8U, 1);
      break;
    default:
      std::cerr << "unsupported color format(" 
                << m_image.data.image.format << ")" << std::endl;
      return RTC::RTC_ERROR;
    }
    switch(m_image.data.image.format){
    case Img::CF_RGB:
      {
        // RGB -> BGR
        char *dst = cvImage->imageData;
        for (unsigned int i=0; i<m_image.data.image.raw_data.length(); i+=3){
          dst[i  ] = m_image.data.image.raw_data[i+2]; 
          dst[i+1] = m_image.data.image.raw_data[i+1]; 
          dst[i+2] = m_image.data.image.raw_data[i  ]; 
        }
        break;
      }
    case Img::CF_GRAY:
      memcpy(cvImage->imageData, 
             m_image.data.image.raw_data.get_buffer(),
             m_image.data.image.raw_data.length());
      break;
    default:
      break;
    }

    char fname[256];
    sprintf(fname, "%s%04d.png", m_basename.c_str(), m_count++);
    cvSaveImage(fname, cvImage);
    
    cvReleaseImage(&cvImage);
  }
  
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t CameraImageSaver::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t CameraImageSaver::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t CameraImageSaver::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t CameraImageSaver::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t CameraImageSaver::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/



extern "C"
{

  void CameraImageSaverInit(RTC::Manager* manager)
  {
    RTC::Properties profile(spec);
    manager->registerFactory(profile,
                             RTC::Create<CameraImageSaver>,
                             RTC::Delete<CameraImageSaver>);
  }

};


