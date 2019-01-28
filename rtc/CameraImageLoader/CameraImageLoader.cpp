// -*- C++ -*-
/*!
 * @file  CameraImageLoader.cpp
 * @brief camera image loader
 * $Date$
 *
 * $Id$
 */

#include <cv.h>
#include <highgui.h>
#include "CameraImageLoader.h"

// Module specification
// <rtc-template block="module_spec">
static const char* spec[] =
  {
    "implementation_id", "CameraImageLoader",
    "type_name",         "CameraImageLoader",
    "description",       "camera image loader",
    "version",           HRPSYS_PACKAGE_VERSION,
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

CameraImageLoader::CameraImageLoader(RTC::Manager* manager)
  : RTC::DataFlowComponentBase(manager),
    // <rtc-template block="initializer">
    m_imageOut("image", m_image),
    // </rtc-template>
    dummy(0)
{
}

CameraImageLoader::~CameraImageLoader()
{
}



RTC::ReturnCode_t CameraImageLoader::onInitialize()
{
  //std::cout << m_profile.instance_name << ": onInitialize()" << std::endl;
  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  
  // </rtc-template>

  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers

  // Set OutPort buffer
    addOutPort("image", m_imageOut);
  
  // Set service provider to Ports
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  
  // </rtc-template>

  RTC::Properties& prop = getProperties();

  return RTC::RTC_OK;
}



/*
RTC::ReturnCode_t CameraImageLoader::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t CameraImageLoader::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t CameraImageLoader::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

RTC::ReturnCode_t CameraImageLoader::onActivated(RTC::UniqueId ec_id)
{
  std::cout << m_profile.instance_name<< ": onActivated(" << ec_id << ")" << std::endl;
  return RTC::RTC_OK;
}

RTC::ReturnCode_t CameraImageLoader::onDeactivated(RTC::UniqueId ec_id)
{
  std::cout << m_profile.instance_name<< ": onDeactivated(" << ec_id << ")" << std::endl;
  return RTC::RTC_OK;
}

RTC::ReturnCode_t CameraImageLoader::onExecute(RTC::UniqueId ec_id)
{
  //std::cout << m_profile.instance_name<< ": onExecute(" << ec_id << ")" << std::endl;
  std::cerr << "image filename: " << std::flush;
  std::string filename;
  std::cin >> filename;

  IplImage *image = cvLoadImage(filename.c_str(), CV_LOAD_IMAGE_COLOR);
  if (!image) {
      std::cerr << m_profile.instance_name << ": failed to load("
                << filename << ")" << std::endl;
      return RTC::RTC_OK;
  }

  m_image.data.image.width = image->width;
  m_image.data.image.height = image->height;
  m_image.data.image.raw_data.length(image->imageSize);
  switch(image->nChannels){
  case 3:
      m_image.data.image.format = Img::CF_RGB;
      {
          // BGR -> RGB
          char *src;
          unsigned char *dst = m_image.data.image.raw_data.get_buffer();
          for (int i=0; i<image->height; i++){
              for (int j=0; j<image->width; j++){
                  src = image->imageData + image->widthStep*i + j*3;
                  dst[2] = src[0];
                  dst[1] = src[1];
                  dst[0] = src[2];
                  dst += 3;
              }
          }
          break;
      }
  case 1:
      m_image.data.image.format = Img::CF_GRAY;
      memcpy(m_image.data.image.raw_data.get_buffer(),
             image->imageData,
             m_image.data.image.raw_data.length());
      break;
  default:
      break;
  }
  
  cvReleaseImage (&image);
  
  m_imageOut.write();
  
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t CameraImageLoader::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t CameraImageLoader::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t CameraImageLoader::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t CameraImageLoader::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t CameraImageLoader::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/



extern "C"
{

  void CameraImageLoaderInit(RTC::Manager* manager)
  {
    RTC::Properties profile(spec);
    manager->registerFactory(profile,
                             RTC::Create<CameraImageLoader>,
                             RTC::Delete<CameraImageLoader>);
  }

};


