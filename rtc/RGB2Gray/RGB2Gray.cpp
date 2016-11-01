// -*- C++ -*-
/*!
 * @file  RGB2Gray.cpp
 * @brief rgb2gray component
 * $Date$
 *
 * $Id$
 */

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include "RGB2Gray.h"

// Module specification
// <rtc-template block="module_spec">
static const char* jpegdecoder_spec[] =
  {
    "implementation_id", "RGB2Gray",
    "type_name",         "RGB2Gray",
    "description",       "rgb2gray component",
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

RGB2Gray::RGB2Gray(RTC::Manager* manager)
  : RTC::DataFlowComponentBase(manager),
    // <rtc-template block="initializer">
    m_rgbIn("rgb",  m_rgb),
    m_grayOut("gray", m_gray),
    // </rtc-template>
    dummy(0)
{
}

RGB2Gray::~RGB2Gray()
{
}



RTC::ReturnCode_t RGB2Gray::onInitialize()
{
  std::cout << m_profile.instance_name << ": onInitialize()" << std::endl;
  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  
  // </rtc-template>

  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("rgbIn", m_rgbIn);

  // Set OutPort buffer
  addOutPort("grayOut", m_grayOut);
  
  // Set service provider to Ports
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  
  // </rtc-template>

  //RTC::Properties& prop = getProperties();

  return RTC::RTC_OK;
}



/*
RTC::ReturnCode_t RGB2Gray::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t RGB2Gray::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t RGB2Gray::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

RTC::ReturnCode_t RGB2Gray::onActivated(RTC::UniqueId ec_id)
{
  std::cout << m_profile.instance_name<< ": onActivated(" << ec_id << ")" << std::endl;
  return RTC::RTC_OK;
}

RTC::ReturnCode_t RGB2Gray::onDeactivated(RTC::UniqueId ec_id)
{
  std::cout << m_profile.instance_name<< ": onDeactivated(" << ec_id << ")" << std::endl;
  return RTC::RTC_OK;
}

RTC::ReturnCode_t RGB2Gray::onExecute(RTC::UniqueId ec_id)
{
    //std::cout << m_profile.instance_name<< ": onExecute(" << ec_id << ")" << std::endl;
  if (m_rgbIn.isNew()){
      m_rgbIn.read();

      Img::ImageData& idat = m_rgb.data.image;

      cv::Mat src(idat.height, idat.width, CV_8UC3, 
		  idat.raw_data.get_buffer());
      cv::Mat dst;
      cv::cvtColor(src, dst, CV_RGB2GRAY);

      m_gray.data.image.width  = idat.width;
      m_gray.data.image.height = idat.height;
      m_gray.data.image.format = Img::CF_GRAY;
      m_gray.data.image.raw_data.length(idat.width*idat.height);
      memcpy(m_gray.data.image.raw_data.get_buffer(),
	     dst.data, idat.width*idat.height);

      m_grayOut.write();
  }
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t RGB2Gray::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t RGB2Gray::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t RGB2Gray::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t RGB2Gray::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t RGB2Gray::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/



extern "C"
{

  void RGB2GrayInit(RTC::Manager* manager)
  {
    RTC::Properties profile(jpegdecoder_spec);
    manager->registerFactory(profile,
                             RTC::Create<RGB2Gray>,
                             RTC::Delete<RGB2Gray>);
  }

};


