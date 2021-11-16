// -*- C++ -*-
/*!
 * @file  RotateImage.cpp
 * @brief rotate image component
 * $Date$
 *
 * $Id$
 */

#include <opencv2/highgui/highgui_c.h>
#include <opencv2/imgproc/imgproc_c.h>
#include "RotateImage.h"

// Module specification
// <rtc-template block="module_spec">
static const char* spec[] =
  {
    "implementation_id", "RotateImage",
    "type_name",         "RotateImage",
    "description",       "rotate image component",
    "version",           HRPSYS_PACKAGE_VERSION,
    "vendor",            "AIST",
    "category",          "example",
    "activity_type",     "DataFlowComponent",
    "max_instance",      "10",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    "conf.default.angle", "0.0",

    ""
  };
// </rtc-template>

RotateImage::RotateImage(RTC::Manager* manager)
  : RTC::DataFlowComponentBase(manager),
    // <rtc-template block="initializer">
    m_originalIn("original",  m_original),
    m_rotatedOut("rotated", m_rotated),
    // </rtc-template>
    m_angle(1.0), m_src(NULL), m_dst(NULL),
    dummy(0)
{
}

RotateImage::~RotateImage()
{
  if (m_src) cvReleaseImage(&m_src);
  if (m_dst) cvReleaseImage(&m_dst);

}



RTC::ReturnCode_t RotateImage::onInitialize()
{
  std::cout << m_profile.instance_name << ": onInitialize()" << std::endl;
  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  bindParameter("angle", m_angle, "0.0");
  
  // </rtc-template>

  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("original", m_originalIn);

  // Set OutPort buffer
  addOutPort("rotated", m_rotatedOut);
  
  // Set service provider to Ports
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  
  // </rtc-template>

  //RTC::Properties& prop = getProperties();

  return RTC::RTC_OK;
}



/*
RTC::ReturnCode_t RotateImage::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t RotateImage::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t RotateImage::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

RTC::ReturnCode_t RotateImage::onActivated(RTC::UniqueId ec_id)
{
  std::cout << m_profile.instance_name<< ": onActivated(" << ec_id << ")" << std::endl;
  return RTC::RTC_OK;
}

RTC::ReturnCode_t RotateImage::onDeactivated(RTC::UniqueId ec_id)
{
  std::cout << m_profile.instance_name<< ": onDeactivated(" << ec_id << ")" << std::endl;
  return RTC::RTC_OK;
}

RTC::ReturnCode_t RotateImage::onExecute(RTC::UniqueId ec_id)
{
    //std::cout << m_profile.instance_name<< ": onExecute(" << ec_id << ")" << std::endl;
  if (m_originalIn.isNew()){
      m_originalIn.read();

      Img::ImageData& idat = m_original.data.image;

      int nchannels = idat.format == Img::CF_GRAY ? 1 : 3;

      if (m_src && (m_src->width != idat.width
		    || m_src->height != idat.height)){
	cvReleaseImage(&m_src);
	cvReleaseImage(&m_dst);
	m_src = m_dst = NULL;
      }
      if (!m_src){
	m_src = cvCreateImage(cvSize(idat.width, idat.height), 
			      IPL_DEPTH_8U, nchannels);
	m_dst = cvCreateImage(cvSize(idat.width, idat.height), 
                              IPL_DEPTH_8U, nchannels);
	m_rotated.data.image.width  = idat.width;
	m_rotated.data.image.height = idat.height;
	m_rotated.data.image.format = idat.format;
	m_rotated.data.image.raw_data.length(idat.width*idat.height*nchannels);
      }

      memcpy(m_src->imageData, idat.raw_data.get_buffer(), 
	     idat.raw_data.length());

      CvMat *rotationMat = cvCreateMat(2,3, CV_32FC1);
      cv2DRotationMatrix(cvPoint2D32f(idat.width/2, idat.height/2), 
                         m_angle*180/M_PI, 1, rotationMat);
      cvWarpAffine(m_src, m_dst, rotationMat);
      cvReleaseMat(&rotationMat);

      memcpy(m_rotated.data.image.raw_data.get_buffer(),
	     m_dst->imageData, m_rotated.data.image.raw_data.length());

      m_rotatedOut.write();
  }
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t RotateImage::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t RotateImage::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t RotateImage::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t RotateImage::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t RotateImage::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/



extern "C"
{

  void RotateImageInit(RTC::Manager* manager)
  {
    RTC::Properties profile(spec);
    manager->registerFactory(profile,
                             RTC::Create<RotateImage>,
                             RTC::Delete<RotateImage>);
  }

};


