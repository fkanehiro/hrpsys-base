// -*- C++ -*-
/*!
 * @file  ResizeImage.cpp
 * @brief resize image component
 * $Date$
 *
 * $Id$
 */

#include <opencv2/highgui/highgui_c.h>
#include <opencv2/imgproc/imgproc_c.h>
#include "ResizeImage.h"

// Module specification
// <rtc-template block="module_spec">
static const char* jpegdecoder_spec[] =
  {
    "implementation_id", "ResizeImage",
    "type_name",         "ResizeImage",
    "description",       "resize image component",
    "version",           HRPSYS_PACKAGE_VERSION,
    "vendor",            "AIST",
    "category",          "example",
    "activity_type",     "DataFlowComponent",
    "max_instance",      "10",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    "conf.default.scale", "1.0",

    ""
  };
// </rtc-template>

ResizeImage::ResizeImage(RTC::Manager* manager)
  : RTC::DataFlowComponentBase(manager),
    // <rtc-template block="initializer">
    m_originalIn("original",  m_original),
    m_resizedOut("resized", m_resized),
    // </rtc-template>
    m_scale(1.0), m_src(NULL), m_dst(NULL),
    dummy(0)
{
}

ResizeImage::~ResizeImage()
{
  if (m_src) cvReleaseImage(&m_src);
  if (m_dst) cvReleaseImage(&m_dst);

}



RTC::ReturnCode_t ResizeImage::onInitialize()
{
  std::cout << m_profile.instance_name << ": onInitialize()" << std::endl;
  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  bindParameter("scale", m_scale, "1.0");
  
  // </rtc-template>

  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("original", m_originalIn);

  // Set OutPort buffer
  addOutPort("resized", m_resizedOut);
  
  // Set service provider to Ports
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  
  // </rtc-template>

  //RTC::Properties& prop = getProperties();

  return RTC::RTC_OK;
}



/*
RTC::ReturnCode_t ResizeImage::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t ResizeImage::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t ResizeImage::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

RTC::ReturnCode_t ResizeImage::onActivated(RTC::UniqueId ec_id)
{
  std::cout << m_profile.instance_name<< ": onActivated(" << ec_id << ")" << std::endl;
  return RTC::RTC_OK;
}

RTC::ReturnCode_t ResizeImage::onDeactivated(RTC::UniqueId ec_id)
{
  std::cout << m_profile.instance_name<< ": onDeactivated(" << ec_id << ")" << std::endl;
  return RTC::RTC_OK;
}

RTC::ReturnCode_t ResizeImage::onExecute(RTC::UniqueId ec_id)
{
    //std::cout << m_profile.instance_name<< ": onExecute(" << ec_id << ")" << std::endl;
  if (m_originalIn.isNew()){
      m_originalIn.read();

      Img::ImageData& idat = m_original.data.image;

      int nchannels = idat.format == Img::CF_GRAY ? 1 : 3;
      int w=idat.width*m_scale, h=idat.height*m_scale;

      if (m_src && (m_src->width != idat.width
		    || m_src->height != idat.height)){
	cvReleaseImage(&m_src);
	cvReleaseImage(&m_dst);
	m_src = m_dst = NULL;
      }
      if (!m_src){
	m_src = cvCreateImage(cvSize(idat.width, idat.height), 
			      IPL_DEPTH_8U, nchannels);
	m_dst = cvCreateImage(cvSize(w,h), IPL_DEPTH_8U, nchannels);
	m_resized.data.image.width  = w;
	m_resized.data.image.height = h;
	m_resized.data.image.format = idat.format;
	m_resized.data.image.raw_data.length(w*h*nchannels);
      }

      memcpy(m_src->imageData, idat.raw_data.get_buffer(), 
	     idat.raw_data.length());

      cvResize(m_src, m_dst, CV_INTER_LINEAR);

      memcpy(m_resized.data.image.raw_data.get_buffer(),
	     m_dst->imageData, m_resized.data.image.raw_data.length());

      m_resizedOut.write();
  }
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t ResizeImage::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t ResizeImage::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t ResizeImage::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t ResizeImage::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t ResizeImage::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/



extern "C"
{

  void ResizeImageInit(RTC::Manager* manager)
  {
    RTC::Properties profile(jpegdecoder_spec);
    manager->registerFactory(profile,
                             RTC::Create<ResizeImage>,
                             RTC::Delete<ResizeImage>);
  }

};


