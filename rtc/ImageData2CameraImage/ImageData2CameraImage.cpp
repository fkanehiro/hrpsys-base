// -*- C++ -*-
/*!
 * @file  ImageData2CameraImage.cpp
 * @brief null component
 * $Date$
 *
 * $Id$
 */

#include "ImageData2CameraImage.h"

// Module specification
// <rtc-template block="module_spec">
static const char* imagedata2cameraimage_spec[] =
  {
    "implementation_id", "ImageData2CameraImage",
    "type_name",         "ImageData2CameraImage",
    "description",       "imagedata2cameraimage component",
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

ImageData2CameraImage::ImageData2CameraImage(RTC::Manager* manager)
  : RTC::DataFlowComponentBase(manager),
    // <rtc-template block="initializer">
    m_dataIn("imageIn", m_data.data.image),
    m_dataOut("imageOut", m_data),
    // </rtc-template>
    dummy(0)
{
}

ImageData2CameraImage::~ImageData2CameraImage()
{
}



RTC::ReturnCode_t ImageData2CameraImage::onInitialize()
{
  std::cout << m_profile.instance_name << ": onInitialize()" << std::endl;
  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  
  // </rtc-template>

  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("imageIn", m_dataIn);

  // Set OutPort buffer
  addOutPort("imageOut", m_dataOut);
  
  // Set service provider to Ports
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  
  // </rtc-template>
  m_data.error_code = 0;

  return RTC::RTC_OK;
}



/*
RTC::ReturnCode_t ImageData2CameraImage::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t ImageData2CameraImage::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t ImageData2CameraImage::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

RTC::ReturnCode_t ImageData2CameraImage::onActivated(RTC::UniqueId ec_id)
{
  std::cout << m_profile.instance_name<< ": onActivated(" << ec_id << ")" << std::endl;
  return RTC::RTC_OK;
}

RTC::ReturnCode_t ImageData2CameraImage::onDeactivated(RTC::UniqueId ec_id)
{
  std::cout << m_profile.instance_name<< ": onDeactivated(" << ec_id << ")" << std::endl;
  return RTC::RTC_OK;
}

RTC::ReturnCode_t ImageData2CameraImage::onExecute(RTC::UniqueId ec_id)
{
  if (m_dataIn.isNew()){
    m_dataIn.read();
    m_dataOut.write();
  }
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t ImageData2CameraImage::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t ImageData2CameraImage::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t ImageData2CameraImage::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t ImageData2CameraImage::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t ImageData2CameraImage::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/



extern "C"
{

  void ImageData2CameraImageInit(RTC::Manager* manager)
  {
    RTC::Properties profile(imagedata2cameraimage_spec);
    manager->registerFactory(profile,
                             RTC::Create<ImageData2CameraImage>,
                             RTC::Delete<ImageData2CameraImage>);
  }

};


