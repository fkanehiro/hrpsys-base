// -*- C++ -*-
/*!
 * @file  UndistortImage.cpp
 * @brief null component
 * $Date$
 *
 * $Id$
 */

#include "UndistortImage.h"

// Module specification
// <rtc-template block="module_spec">
static const char* cameraimageviewercomponent_spec[] =
{
    "implementation_id", "UndistortImage",
    "type_name",         "UndistortImage",
    "description",       "camera image undistortion component",
    "version",           HRPSYS_PACKAGE_VERSION,
    "vendor",            "AIST",
    "category",          "example",
    "activity_type",     "DataFlowComponent",
    "max_instance",      "10",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    "conf.default.calibFile", "camera.xml",

    ""
};
// </rtc-template>

UndistortImage::UndistortImage(RTC::Manager* manager)
    : RTC::DataFlowComponentBase(manager),
      // <rtc-template block="initializer">
      m_imageIn("imageIn", m_image),
      m_imageOut("imageOut", m_image),
      // </rtc-template>
      m_cvImage(NULL),
      m_intrinsic(NULL),
      m_distortion(NULL),
      dummy(0)
{
}

UndistortImage::~UndistortImage()
{
}



RTC::ReturnCode_t UndistortImage::onInitialize()
{
    std::cout << m_profile.instance_name << ": onInitialize()" << std::endl;
    // <rtc-template block="bind_config">
    // Bind variables and configuration variable
    bindParameter("calibFile", m_calibFile, "camera.xml");
  
    // </rtc-template>

    // Registration: InPort/OutPort/Service
    // <rtc-template block="registration">
    // Set InPort buffers
    addInPort("imageIn", m_imageIn);

    // Set OutPort buffer
    addOutPort("imageOut", m_imageOut);
  
    // Set service provider to Ports
  
    // Set service consumers to Ports
  
    // Set CORBA Service Ports
  
    // </rtc-template>

    RTC::Properties& prop = getProperties();

    return RTC::RTC_OK;
}



/*
  RTC::ReturnCode_t UndistortImage::onFinalize()
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t UndistortImage::onStartup(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t UndistortImage::onShutdown(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

RTC::ReturnCode_t UndistortImage::onActivated(RTC::UniqueId ec_id)
{
    std::cout << m_profile.instance_name<< ": onActivated(" << ec_id << ")" << std::endl;

    CvFileStorage *fs 
        = cvOpenFileStorage (m_calibFile.c_str(), 0, CV_STORAGE_READ);
    if (!fs){
        std::cerr << m_profile.instance_name << ": can't open "
                  << m_calibFile << std::endl;
        return RTC::RTC_ERROR;
    }
    CvFileNode *param = cvGetFileNodeByName (fs, NULL, "intrinsic");
    m_intrinsic = (CvMat *) cvRead (fs, param);
    param = cvGetFileNodeByName (fs, NULL, "distortion");
    m_distortion = (CvMat *) cvRead (fs, param);
    cvReleaseFileStorage (&fs);

    return RTC::RTC_OK;
}

RTC::ReturnCode_t UndistortImage::onDeactivated(RTC::UniqueId ec_id)
{
    std::cout << m_profile.instance_name<< ": onDeactivated(" << ec_id << ")" << std::endl;
    if (m_cvImage) {
        cvReleaseImage(&m_cvImage);
        m_cvImage = NULL;
    }
    if (m_intrinsic) cvReleaseMat (&m_intrinsic);
    if (m_distortion) cvReleaseMat (&m_distortion);
    
    return RTC::RTC_OK;
}

RTC::ReturnCode_t UndistortImage::onExecute(RTC::UniqueId ec_id)
{
    //std::cout << m_profile.instance_name<< ": onExecute(" << ec_id << ")"  << std::endl;

    if (!m_imageIn.isNew()) return RTC::RTC_OK;

    m_imageIn.read();

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
    
    
    IplImage *dst_img = cvCloneImage (m_cvImage);
    cvUndistort2 (m_cvImage, dst_img, m_intrinsic, m_distortion);

    switch(m_image.data.image.format){
    case Img::CF_RGB:
        {
            // BGR -> RGB
            char *src = dst_img->imageData;
            for (unsigned int i=0; i<m_image.data.image.raw_data.length(); i+=3){
                m_image.data.image.raw_data[i+2] = src[i  ]; 
                m_image.data.image.raw_data[i+1] = src[i+1]; 
                m_image.data.image.raw_data[i  ] = src[i+2]; 
            }
            break;
        }
    case Img::CF_GRAY:
        memcpy(m_image.data.image.raw_data.get_buffer(),
               dst_img->imageData,
               m_image.data.image.raw_data.length());
        break;
    default:
        break;
    }

    cvReleaseImage (&dst_img);

    m_imageOut.write();

    return RTC::RTC_OK;
}

/*
  RTC::ReturnCode_t UndistortImage::onAborting(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t UndistortImage::onError(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t UndistortImage::onReset(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t UndistortImage::onStateUpdate(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t UndistortImage::onRateChanged(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/



extern "C"
{

    void UndistortImageInit(RTC::Manager* manager)
    {
        RTC::Properties profile(cameraimageviewercomponent_spec);
        manager->registerFactory(profile,
                                 RTC::Create<UndistortImage>,
                                 RTC::Delete<UndistortImage>);
    }

};


