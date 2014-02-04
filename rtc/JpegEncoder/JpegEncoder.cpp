// -*- C++ -*-
/*!
 * @file  JpegEncoder.cpp
 * @brief null component
 * $Date$
 *
 * $Id$
 */

#include <cv.h>
#include <highgui.h>
#include "JpegEncoder.h"

// Module specification
// <rtc-template block="module_spec">
static const char* jpegdecoder_spec[] =
  {
    "implementation_id", "JpegEncoder",
    "type_name",         "JpegEncoder",
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

JpegEncoder::JpegEncoder(RTC::Manager* manager)
  : RTC::DataFlowComponentBase(manager),
    // <rtc-template block="initializer">
    m_decodedIn("decoded",  m_decoded),
    m_encodedOut("encoded", m_encoded),
    // </rtc-template>
    dummy(0)
{
}

JpegEncoder::~JpegEncoder()
{
}



RTC::ReturnCode_t JpegEncoder::onInitialize()
{
  std::cout << m_profile.instance_name << ": onInitialize()" << std::endl;
  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  
  // </rtc-template>

  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("decodedIn", m_decodedIn);

  // Set OutPort buffer
  addOutPort("encodedOut", m_encodedOut);
  
  // Set service provider to Ports
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  
  // </rtc-template>

  //RTC::Properties& prop = getProperties();

  return RTC::RTC_OK;
}



/*
RTC::ReturnCode_t JpegEncoder::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t JpegEncoder::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t JpegEncoder::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

RTC::ReturnCode_t JpegEncoder::onActivated(RTC::UniqueId ec_id)
{
  std::cout << m_profile.instance_name<< ": onActivated(" << ec_id << ")" << std::endl;
  return RTC::RTC_OK;
}

RTC::ReturnCode_t JpegEncoder::onDeactivated(RTC::UniqueId ec_id)
{
  std::cout << m_profile.instance_name<< ": onDeactivated(" << ec_id << ")" << std::endl;
  return RTC::RTC_OK;
}

RTC::ReturnCode_t JpegEncoder::onExecute(RTC::UniqueId ec_id)
{
    //std::cout << m_profile.instance_name<< ": onExecute(" << ec_id << ")" << std::endl;
  if (m_decodedIn.isNew()){
      m_decodedIn.read();

      Img::ImageData& idat = m_decoded.data.image;

      // RGB -> BGR
      uchar r,g,b, *raw=idat.raw_data.get_buffer();
      for (int i=0; i<idat.raw_data.length(); i+=3, raw+=3){
	r = raw[0]; g = raw[1]; b = raw[2];
	raw[0] = b; raw[1] = g; raw[2] = r;
      }

      cv::Mat src(idat.height, idat.width, CV_8UC3, 
		  idat.raw_data.get_buffer());
      std::vector<uchar>buf;
      std::vector<int> param = std::vector<int>(2);
      param[0] = CV_IMWRITE_JPEG_QUALITY;
      param[1] = 95;
      imencode(".jpg", src, buf, param);

      m_encoded.data.image.raw_data.length(buf.size());
      unsigned char *dst = m_encoded.data.image.raw_data.get_buffer();
      memcpy(dst, &buf[0], buf.size());

      m_encodedOut.write();
  }
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t JpegEncoder::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t JpegEncoder::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t JpegEncoder::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t JpegEncoder::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t JpegEncoder::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/



extern "C"
{

  void JpegEncoderInit(RTC::Manager* manager)
  {
    RTC::Properties profile(jpegdecoder_spec);
    manager->registerFactory(profile,
                             RTC::Create<JpegEncoder>,
                             RTC::Delete<JpegEncoder>);
  }

};


