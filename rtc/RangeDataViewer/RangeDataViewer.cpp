// -*- C++ -*-
/*!
 * @file  RangeDataViewer.cpp
 * @brief null component
 * $Date$
 *
 * $Id$
 */

#include "RangeDataViewer.h"

// Module specification
// <rtc-template block="module_spec">
static const char* cameraimageviewercomponent_spec[] =
  {
    "implementation_id", "RangeDataViewer",
    "type_name",         "RangeDataViewer",
    "description",       "range data viewer component",
    "version",           HRPSYS_PACKAGE_VERSION,
    "vendor",            "AIST",
    "category",          "example",
    "activity_type",     "DataFlowComponent",
    "max_instance",      "10",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    "conf.default.maxRange", "2.0",
    

    ""
  };
// </rtc-template>

RangeDataViewer::RangeDataViewer(RTC::Manager* manager)
  : RTC::DataFlowComponentBase(manager),
    // <rtc-template block="initializer">
    m_rangeIn("rangeIn", m_range),
    // </rtc-template>
    m_cvImage(NULL),
    dummy(0)
{
}

RangeDataViewer::~RangeDataViewer()
{
}



RTC::ReturnCode_t RangeDataViewer::onInitialize()
{
  std::cout << m_profile.instance_name << ": onInitialize()" << std::endl;
  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  bindParameter("maxRange", m_maxRange, "2.0");
  
  // </rtc-template>

  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("rangeIn", m_rangeIn);

  // Set OutPort buffer
  
  // Set service provider to Ports
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  
  // </rtc-template>

  RTC::Properties& prop = getProperties();

  return RTC::RTC_OK;
}



/*
  RTC::ReturnCode_t RangeDataViewer::onFinalize()
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t RangeDataViewer::onStartup(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t RangeDataViewer::onShutdown(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

#define WSIZE 640
RTC::ReturnCode_t RangeDataViewer::onActivated(RTC::UniqueId ec_id)
{
  std::cout << m_profile.instance_name<< ": onActivated(" << ec_id << ")" << std::endl;
  m_cvImage = cvCreateImage(cvSize(WSIZE, WSIZE), IPL_DEPTH_8U, 3);
  cvNamedWindow("Range",CV_WINDOW_AUTOSIZE);
  return RTC::RTC_OK;
}

RTC::ReturnCode_t RangeDataViewer::onDeactivated(RTC::UniqueId ec_id)
{
  std::cout << m_profile.instance_name<< ": onDeactivated(" << ec_id << ")" << std::endl;
  if (m_cvImage) {
    cvReleaseImage(&m_cvImage);
    m_cvImage = NULL;
  }
  cvDestroyWindow("Range");
  return RTC::RTC_OK;
}

RTC::ReturnCode_t RangeDataViewer::onExecute(RTC::UniqueId ec_id)
{
  //std::cout << m_profile.instance_name<< ": onExecute(" << ec_id << ")"  << std::endl;

  if (1/*m_rangeIn.isNew()*/){
    do {
      m_rangeIn.read();
    }while(m_rangeIn.isNew());

    cvSetZero(m_cvImage);

    CvPoint center = cvPoint(WSIZE/2,WSIZE/2);
    CvScalar green = cvScalar(0,255,0);
    double th,d,x,y;
#if 0
    std::cout << "minAngle:" << m_range.config.minAngle << std::endl;
    std::cout << "maxAngle:" << m_range.config.maxAngle << std::endl;
    std::cout << "angularRes:" << m_range.config.angularRes << std::endl;
    std::cout << "minRange:" << m_range.config.minRange << std::endl;
    std::cout << "maxRange:" << m_range.config.maxRange << std::endl;
    std::cout << "rangeRes:" << m_range.config.rangeRes << std::endl;
    std::cout << "frequency:" << m_range.config.frequency << std::endl;
    std::cout << "ndata = " << m_range.ranges.length() << std::endl;
#endif
    for (unsigned int i=0; i<m_range.ranges.length(); i++){
      d = m_range.ranges[i];
      if (isinf(d)) continue;
      th = m_range.config.minAngle + m_range.config.angularRes*i;
      x = -d*sin(th)/m_maxRange*WSIZE/2 + WSIZE/2;
      y = -d*cos(th)/m_maxRange*WSIZE/2 + WSIZE/2;
      cvLine(m_cvImage, center, cvPoint(x, y), green, 1, 8, 0);
    }

    cvShowImage("Range",m_cvImage);
    cvWaitKey(10);
  }

  return RTC::RTC_OK;
}

/*
  RTC::ReturnCode_t RangeDataViewer::onAborting(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t RangeDataViewer::onError(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t RangeDataViewer::onReset(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t RangeDataViewer::onStateUpdate(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t RangeDataViewer::onRateChanged(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/



extern "C"
{

  void RangeDataViewerInit(RTC::Manager* manager)
  {
    RTC::Properties profile(cameraimageviewercomponent_spec);
    manager->registerFactory(profile,
			     RTC::Create<RangeDataViewer>,
			     RTC::Delete<RangeDataViewer>);
  }

};


