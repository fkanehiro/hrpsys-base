// -*- C++ -*-
/*!
 * @file  ColorExtractor.cpp
 * @brief rotate image component
 * $Date$
 *
 * $Id$
 */

#include <highgui.h>
#include "ColorExtractor.h"
#include "hrpsys/util/VectorConvert.h"

// Module specification
// <rtc-template block="module_spec">
static const char* spec[] =
  {
    "implementation_id", "ColorExtractor",
    "type_name",         "ColorExtractor",
    "description",       "rotate image component",
    "version",           HRPSYS_PACKAGE_VERSION,
    "vendor",            "AIST",
    "category",          "example",
    "activity_type",     "DataFlowComponent",
    "max_instance",      "10",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    "conf.default.minPixels", "0",
    "conf.default.rgbRegion", "0,0,0,0,0,0",

    ""
  };
// </rtc-template>

ColorExtractor::ColorExtractor(RTC::Manager* manager)
  : RTC::DataFlowComponentBase(manager),
    // <rtc-template block="initializer">
    m_originalIn("original",  m_original),
    m_resultOut("result", m_result),
    m_posOut("pos", m_pos),
    // </rtc-template>
    m_img(NULL),
    dummy(0)
{
}

ColorExtractor::~ColorExtractor()
{
  if (m_img) cvReleaseImage(&m_img);

}



RTC::ReturnCode_t ColorExtractor::onInitialize()
{
  std::cout << m_profile.instance_name << ": onInitialize()" << std::endl;
  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  bindParameter("minPixels", m_minPixels, "0");
  bindParameter("rgbRegion", m_rgbRegion, "0,0,0,0,0,0");
  
  // </rtc-template>

  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("original", m_originalIn);

  // Set OutPort buffer
  addOutPort("result", m_resultOut);
  
  // Set service provider to Ports
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  
  // </rtc-template>

  //RTC::Properties& prop = getProperties();

  return RTC::RTC_OK;
}



/*
RTC::ReturnCode_t ColorExtractor::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t ColorExtractor::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t ColorExtractor::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

RTC::ReturnCode_t ColorExtractor::onActivated(RTC::UniqueId ec_id)
{
  std::cout << m_profile.instance_name<< ": onActivated(" << ec_id << ")" << std::endl;
  return RTC::RTC_OK;
}

RTC::ReturnCode_t ColorExtractor::onDeactivated(RTC::UniqueId ec_id)
{
  std::cout << m_profile.instance_name<< ": onDeactivated(" << ec_id << ")" << std::endl;
  return RTC::RTC_OK;
}

RTC::ReturnCode_t ColorExtractor::onExecute(RTC::UniqueId ec_id)
{
    //std::cout << m_profile.instance_name<< ": onExecute(" << ec_id << ")" << std::endl;
  if (m_originalIn.isNew()){
      m_originalIn.read();

      Img::ImageData& idat = m_original.data.image;

      if (m_img && (m_img->width != idat.width
		    || m_img->height != idat.height)){
	cvReleaseImage(&m_img);
	m_img = NULL;
      }
      if (!m_img){
	m_img = cvCreateImage(cvSize(idat.width, idat.height), 
			      IPL_DEPTH_8U, 3);
	m_result.data.image.width  = idat.width;
	m_result.data.image.height = idat.height;
	m_result.data.image.format = idat.format;
	m_result.data.image.raw_data.length(idat.width*idat.height*3);
      }

      // RGB -> BGR
      unsigned char *rtm=idat.raw_data.get_buffer();
      char *cv;
      for (int i=0; i<idat.height; i++){
        for (int j=0; j<idat.width; j++){
          cv = m_img->imageData + m_img->widthStep*i + j*3;
          cv[0] = rtm[2];
          cv[1] = rtm[1];
          cv[2] = rtm[0];
          rtm += 3;
        }
      }
      
      // processing start
      int npixel=0, cx=0, cy=0;
      unsigned char b,g,r;
      for (int i=0; i<idat.height; i++){
        for (int j=0; j<idat.width; j++){
          cv = m_img->imageData + m_img->widthStep*i + j*3;
          b = cv[0]; g = cv[1]; r = cv[2];
          if (r > m_rgbRegion[0] && g > m_rgbRegion[1] && b > m_rgbRegion[2]
              && r < m_rgbRegion[3] && g < m_rgbRegion[4] && b < m_rgbRegion[5]){
            cx += j;
            cy += i;
            npixel++;
          }else{
          }
        }
      }
      if (npixel > 10){
        cx /= npixel;
        cy /= npixel;
        //printf("cx=%d, cy=%d, npixel=%d\n", cx, cy, npixel);
        cvCircle(m_img, cvPoint(cx, cy), sqrt(npixel), CV_RGB(0,0,255), 6, 8, 0);
        m_pos.tm = m_original.tm;
        m_pos.data.x = cx;
        m_pos.data.y = cy;
        m_posOut.write();
      }
      // processing end
      
      // BGR -> RGB
      rtm = m_result.data.image.raw_data.get_buffer();
      for (int i=0; i<idat.height; i++){
        for (int j=0; j<idat.width; j++){
          cv = m_img->imageData + m_img->widthStep*i + j*3;
          rtm[2] = cv[0]; 
          rtm[1] = cv[1]; 
          rtm[0] = cv[2];
          rtm += 3;
        }
      }

      m_resultOut.write();
  }
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t ColorExtractor::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t ColorExtractor::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t ColorExtractor::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t ColorExtractor::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t ColorExtractor::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/



extern "C"
{

  void ColorExtractorInit(RTC::Manager* manager)
  {
    RTC::Properties profile(spec);
    manager->registerFactory(profile,
                             RTC::Create<ColorExtractor>,
                             RTC::Delete<ColorExtractor>);
  }

};


