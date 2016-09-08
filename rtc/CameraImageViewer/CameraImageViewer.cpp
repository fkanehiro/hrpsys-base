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
static const char* cameraimageviewercomponent_spec[] =
{
    "implementation_id", "CameraImageViewer",
    "type_name",         "CameraImageViewer",
    "description",       "camera image viewer component",
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
        if (m_cvImage && (m_image.data.image.width != m_cvImage->width 
                          || m_image.data.image.height != m_cvImage->height)){
            cvReleaseImage(&m_cvImage);
            m_cvImage = NULL;
        }
        if (!m_cvImage){
            switch (m_image.data.image.format){
            case Img::CF_RGB:
                m_cvImage = cvCreateImage(cvSize(m_image.data.image.width,
                                                 m_image.data.image.height),
                                          IPL_DEPTH_8U, 3);
                break;
            case Img::CF_GRAY:
                m_cvImage = cvCreateImage(cvSize(m_image.data.image.width,
                                                 m_image.data.image.height),
                                          IPL_DEPTH_8U, 1);
                break;
            default:
                std::cerr << "unsupported color format(" 
                          << m_image.data.image.format << ")" << std::endl;
                return RTC::RTC_ERROR;
            }
        }
        switch(m_image.data.image.format){
        case Img::CF_RGB:
        {
            // RGB -> BGR
            char *dst = m_cvImage->imageData;
            for (unsigned int i=0; i<m_image.data.image.raw_data.length(); i+=3){
                dst[i  ] = m_image.data.image.raw_data[i+2]; 
                dst[i+1] = m_image.data.image.raw_data[i+1]; 
                dst[i+2] = m_image.data.image.raw_data[i  ]; 
            }
            break;
        }
        case Img::CF_GRAY:
            memcpy(m_cvImage->imageData, 
                   m_image.data.image.raw_data.get_buffer(),
                   m_image.data.image.raw_data.length());
            break;
        default:
            break;
        }
        cvShowImage("Image",m_cvImage);
        cvWaitKey(10);
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
        RTC::Properties profile(cameraimageviewercomponent_spec);
        manager->registerFactory(profile,
                                 RTC::Create<CameraImageViewer>,
                                 RTC::Delete<CameraImageViewer>);
    }

};


