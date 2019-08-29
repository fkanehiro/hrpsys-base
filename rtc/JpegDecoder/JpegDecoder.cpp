// -*- C++ -*-
/*!
 * @file  JpegDecoder.cpp
 * @brief null component
 * $Date$
 *
 * $Id$
 */

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include "JpegDecoder.h"

// Module specification
// <rtc-template block="module_spec">
static const char* jpegdecoder_spec[] =
  {
    "implementation_id", "JpegDecoder",
    "type_name",         "JpegDecoder",
    "description",       "null component",
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

      int flags = (idat.format == Img::CF_GRAY_JPEG) ? CV_LOAD_IMAGE_GRAYSCALE : CV_LOAD_IMAGE_COLOR;
      cv::Mat image = cv::imdecode(cv::Mat(buf), flags);

      if (idat.format == Img::CF_GRAY_JPEG){
	m_decoded.data.image.format = Img::CF_GRAY;
	m_decoded.data.image.raw_data.length(image.cols*image.rows);
      }else{
	m_decoded.data.image.format = Img::CF_RGB;
	m_decoded.data.image.raw_data.length(image.cols*image.rows*3);
      }
      m_decoded.data.image.width = image.cols;
      m_decoded.data.image.height = image.rows;
      unsigned char *src = image.data;
      unsigned char *dst = m_decoded.data.image.raw_data.get_buffer();
      if (idat.format == Img::CF_GRAY_JPEG){
	memcpy(dst, src, m_decoded.data.image.raw_data.length());
      }else{
	for (int i=0; i<image.rows; i++){
          for (int j=0; j<image.cols; j++){
	    // BGR -> RGB
	    *dst++ = src[2];
	    *dst++ = src[1];
	    *dst++ = src[0];
	    src+=3;
          }
	}
      }

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


