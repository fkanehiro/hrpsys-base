// -*- C++ -*-
/*!
 * @file  ExtractCameraImage.cpp
 * @brief null component
 * $Date$
 *
 * $Id$
 */

#include "ExtractCameraImage.h"

// Module specification
// <rtc-template block="module_spec">
static const char* extractcameraimage_spec[] =
  {
    "implementation_id", "ExtractCameraImage",
    "type_name",         "ExtractCameraImage",
    "description",       "extractcameraimage component",
    "version",           HRPSYS_PACKAGE_VERSION,
    "vendor",            "AIST",
    "category",          "example",
    "activity_type",     "DataFlowComponent",
    "max_instance",      "10",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    "conf.default.index","0",

    ""
  };
// </rtc-template>

ExtractCameraImage::ExtractCameraImage(RTC::Manager* manager)
  : RTC::DataFlowComponentBase(manager),
    // <rtc-template block="initializer">
    m_imagesIn("images", m_images),
    m_imageOut("image", m_image),
    // </rtc-template>
    m_index(0),
    dummy(0)
{
}

ExtractCameraImage::~ExtractCameraImage()
{
}



RTC::ReturnCode_t ExtractCameraImage::onInitialize()
{
  std::cout << m_profile.instance_name << ": onInitialize()" << std::endl;
  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  bindParameter("index", m_index, "0");
  
  // </rtc-template>

  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("imagesIn", m_imagesIn);

  // Set OutPort buffer
  addOutPort("imageOut", m_imageOut);
  
  // Set service provider to Ports
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  
  // </rtc-template>

  return RTC::RTC_OK;
}



/*
RTC::ReturnCode_t ExtractCameraImage::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t ExtractCameraImage::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t ExtractCameraImage::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

RTC::ReturnCode_t ExtractCameraImage::onActivated(RTC::UniqueId ec_id)
{
  std::cout << m_profile.instance_name<< ": onActivated(" << ec_id << ")" << std::endl;
  return RTC::RTC_OK;
}

RTC::ReturnCode_t ExtractCameraImage::onDeactivated(RTC::UniqueId ec_id)
{
  std::cout << m_profile.instance_name<< ": onDeactivated(" << ec_id << ")" << std::endl;
  return RTC::RTC_OK;
}

RTC::ReturnCode_t ExtractCameraImage::onExecute(RTC::UniqueId ec_id)
{
  if (m_imagesIn.isNew()){
    m_imagesIn.read();
    if (m_images.data.image_seq.length() > m_index){
      m_image.tm = m_images.tm;
      m_image.data = m_images.data.image_seq[m_index];
      m_image.error_code = m_images.error_code;
      m_imageOut.write();
    }else{
      std::cerr << m_profile.instance_name << ": invalid index of image(" << m_index 
		<< "), length of images = " << m_images.data.image_seq.length() << std::endl;
    }
  }
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t ExtractCameraImage::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t ExtractCameraImage::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t ExtractCameraImage::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t ExtractCameraImage::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t ExtractCameraImage::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/



extern "C"
{

  void ExtractCameraImageInit(RTC::Manager* manager)
  {
    RTC::Properties profile(extractcameraimage_spec);
    manager->registerFactory(profile,
                             RTC::Create<ExtractCameraImage>,
                             RTC::Delete<ExtractCameraImage>);
  }

};


