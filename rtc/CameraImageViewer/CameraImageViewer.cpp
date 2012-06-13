// -*- C++ -*-
/*!
 * @file  CameraImageViewer.cpp
 * @brief null component
 * $Date$
 *
 * $Id$
 */

#include "CameraImageViewer.h"

// Module specification
// <rtc-template block="module_spec">
static const char* nullcomponent_spec[] =
{
    "implementation_id", "CameraImageViewer",
    "type_name",         "CameraImageViewer",
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

CameraImageViewer::CameraImageViewer(RTC::Manager* manager)
    : RTC::DataFlowComponentBase(manager),
      // <rtc-template block="initializer">
      m_imageIn("imageIn", m_image),
      // </rtc-template>
      m_cvImage(NULL),
      dummy(0)
{
}

CameraImageViewer::~CameraImageViewer()
{
}



RTC::ReturnCode_t CameraImageViewer::onInitialize()
{
    std::cout << m_profile.instance_name << ": onInitialize()" << std::endl;
    // <rtc-template block="bind_config">
    // Bind variables and configuration variable
  
    // </rtc-template>

    // Registration: InPort/OutPort/Service
    // <rtc-template block="registration">
    // Set InPort buffers
    addInPort("imageIn", m_imageIn);

    // Set OutPort buffer
  
    // Set service provider to Ports
  
    // Set service consumers to Ports
  
    // Set CORBA Service Ports
  
    // </rtc-template>

    RTC::Properties& prop = getProperties();

    return RTC::RTC_OK;
}



/*
  RTC::ReturnCode_t CameraImageViewer::onFinalize()
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t CameraImageViewer::onStartup(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t CameraImageViewer::onShutdown(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

RTC::ReturnCode_t CameraImageViewer::onActivated(RTC::UniqueId ec_id)
{
    std::cout << m_profile.instance_name<< ": onActivated(" << ec_id << ")" << std::endl;
    cvNamedWindow("Image",CV_WINDOW_AUTOSIZE);
    return RTC::RTC_OK;
}

RTC::ReturnCode_t CameraImageViewer::onDeactivated(RTC::UniqueId ec_id)
{
    std::cout << m_profile.instance_name<< ": onDeactivated(" << ec_id << ")" << std::endl;
    if (m_cvImage) {
        cvReleaseImage(&m_cvImage);
        m_cvImage = NULL;
    }
    cvDestroyWindow("Image");
    return RTC::RTC_OK;
}

RTC::ReturnCode_t CameraImageViewer::onExecute(RTC::UniqueId ec_id)
{
    //std::cout << m_profile.instance_name<< ": onExecute(" << ec_id << ")"  << std::endl;

    if (m_imageIn.isNew()){
        do {
            m_imageIn.read();
        }while(m_imageIn.isNew());
        if (!m_cvImage){
            m_cvImage = cvCreateImage(cvSize(m_image.data.image.width,
                                             m_image.data.image.height),
                                      IPL_DEPTH_8U, 3);
        }
        memcpy(m_cvImage->imageData, m_image.data.image.raw_data.get_buffer(),
               m_image.data.image.raw_data.length());
        cvShowImage("Image",m_cvImage);
    }

    return RTC::RTC_OK;
}

/*
  RTC::ReturnCode_t CameraImageViewer::onAborting(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t CameraImageViewer::onError(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t CameraImageViewer::onReset(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t CameraImageViewer::onStateUpdate(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t CameraImageViewer::onRateChanged(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/



extern "C"
{

    void CameraImageViewerInit(RTC::Manager* manager)
    {
        RTC::Properties profile(nullcomponent_spec);
        manager->registerFactory(profile,
                                 RTC::Create<CameraImageViewer>,
                                 RTC::Delete<CameraImageViewer>);
    }

};


