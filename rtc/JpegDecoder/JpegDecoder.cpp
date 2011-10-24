// -*- C++ -*-
/*!
 * @file  JpegDecoder.cpp
 * @brief null component
 * $Date$
 *
 * $Id$
 */

#include <cv.h>
#include <highgui.h>
#include "JpegDecoder.h"

// Module specification
// <rtc-template block="module_spec">
static const char* jpegdecoder_spec[] =
  {
    "implementation_id", "JpegDecoder",
    "type_name",         "JpegDecoder",
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

JpegDecoder::JpegDecoder(RTC::Manager* manager)
  : RTC::DataFlowComponentBase(manager),
    // <rtc-template block="initializer">
    m_encodedIn("encoded",  m_encoded),
    m_decodedOut("decoded", m_decoded),
    // </rtc-template>
    dummy(0)
{
}

JpegDecoder::~JpegDecoder()
{
}



RTC::ReturnCode_t JpegDecoder::onInitialize()
{
  std::cout << m_profile.instance_name << ": onInitialize()" << std::endl;
  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  
  // </rtc-template>

  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("encodedIn", m_encodedIn);

  // Set OutPort buffer
  addOutPort("decodedOut", m_decodedOut);
  
  // Set service provider to Ports
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  
  // </rtc-template>

  //RTC::Properties& prop = getProperties();

  return RTC::RTC_OK;
}



/*
RTC::ReturnCode_t JpegDecoder::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t JpegDecoder::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t JpegDecoder::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

RTC::ReturnCode_t JpegDecoder::onActivated(RTC::UniqueId ec_id)
{
  std::cout << m_profile.instance_name<< ": onActivated(" << ec_id << ")" << std::endl;
  return RTC::RTC_OK;
}

RTC::ReturnCode_t JpegDecoder::onDeactivated(RTC::UniqueId ec_id)
{
  std::cout << m_profile.instance_name<< ": onDeactivated(" << ec_id << ")" << std::endl;
  return RTC::RTC_OK;
}

RTC::ReturnCode_t JpegDecoder::onExecute(RTC::UniqueId ec_id)
{
    //std::cout << m_profile.instance_name<< ": onExecute(" << ec_id << ")" << std::endl;
  if (m_encodedIn.isNew()){
      m_encodedIn.read();

      if (m_encoded.error_code != 0) {
          return RTC::RTC_OK;
      }

      Img::ImageData& idat = m_encoded.data.image;

      std::vector<uchar>buf;
      int len = idat.raw_data.length();
      buf.resize(idat.raw_data.length());
      memcpy(&buf[0], (void *)&(idat.raw_data[0]), len);

      cv::Mat image = cv::imdecode(cv::Mat(buf), 1);

      std::cout << "(" << image.cols << "," << image.rows << ")" << std::endl; 
      m_decoded.data.image.raw_data.length(image.cols*image.rows);
      m_decoded.data.image.width = image.cols;
      m_decoded.data.image.height = image.rows;
      m_decoded.data.image.format = Img::CF_RGB;
      memcpy(m_decoded.data.image.raw_data.get_buffer(),
             image.data,
             m_decoded.data.image.raw_data.length());

      m_decodedOut.write();
  }
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t JpegDecoder::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t JpegDecoder::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t JpegDecoder::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t JpegDecoder::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t JpegDecoder::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/



extern "C"
{

  void JpegDecoderInit(RTC::Manager* manager)
  {
    RTC::Properties profile(jpegdecoder_spec);
    manager->registerFactory(profile,
                             RTC::Create<JpegDecoder>,
                             RTC::Delete<JpegDecoder>);
  }

};


