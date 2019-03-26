// -*- C++ -*-
/*!
 * @file  VideoCapture.cpp
 * @brief video capture component
 * $Date$
 *
 * $Id$
 */

#include "hrpsys/util/VectorConvert.h"
#include "VideoCapture.h"

// Module specification
// <rtc-template block="module_spec">
static const char* videocapture_spec[] =
  {
    "implementation_id", "VideoCapture",
    "type_name",         "VideoCapture",
    "description",       "video capture component",
    "version",           HRPSYS_PACKAGE_VERSION,
    "vendor",            "JSK",
    "category",          "example",
    "activity_type",     "DataFlowComponent",
    "max_instance",      "10",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    "conf.default.initialMode", "continuous",
    "conf.default.devIds", "0",
    "conf.default.width", "640",
    "conf.default.height", "480",
    "conf.default.frameRate", "1",

    ""
  };
// </rtc-template>

VideoCapture::VideoCapture(RTC::Manager* manager)
  : RTC::DataFlowComponentBase(manager),
    // <rtc-template block="initializer">
    m_MultiCameraImagesOut ("MultiCameraImages", m_MultiCameraImages),
    m_CameraImageOut ("CameraImage", m_CameraImage),
    m_CameraCaptureServicePort("CameraCaptureService"),
    m_CameraCaptureService(this),
    // </rtc-template>
    m_mode(CONTINUOUS),
    m_needToReactivate(false)
{
}

VideoCapture::~VideoCapture()
{
}



RTC::ReturnCode_t VideoCapture::onInitialize()
{
  std::cout << m_profile.instance_name << ": onInitialize()" << std::endl;

  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  bindParameter("initialMode", m_initialMode, "continuous");
  bindParameter("devIds", m_devIds, "0");
  bindParameter("width", m_width, "640");
  bindParameter("height", m_height, "480");
  bindParameter("frameRate", m_frameRate, "1");
  
  // </rtc-template>

  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers

  // Set OutPort buffer
  if (m_devIds.size() == 1){
    addOutPort ("CameraImage", m_CameraImageOut);
  }else{
    addOutPort ("MultiCameraImages", m_MultiCameraImagesOut);
  }
  
  // Set service provider to Ports
  m_CameraCaptureServicePort.registerProvider("service0", "CameraCaptureService", m_CameraCaptureService);
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  addPort(m_CameraCaptureServicePort);
  
  // </rtc-template>

  return RTC::RTC_OK;
}



/*
RTC::ReturnCode_t VideoCapture::onFinalize()
{
  return RTC::RTC_OK;
}
*/

RTC::ReturnCode_t VideoCapture::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t VideoCapture::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

RTC::ReturnCode_t VideoCapture::onActivated(RTC::UniqueId ec_id)
{
  std::cout << m_profile.instance_name<< ": onActivated(" << ec_id << ")" << std::endl;
  m_tOld = (double)(coil::gettimeofday());
  if (m_initialMode == "continuous"){
    m_mode = CONTINUOUS;
  }else{
    m_mode = SLEEP;
  }

  if (m_devIds.size() == 1){
    std::cout << "** devId:" << m_devIds[0] << std::endl;
    v4l_capture *cam = new v4l_capture ();
    if (cam->init(m_width, m_height, m_devIds[0]) != 0) return RTC::RTC_ERROR;
    m_cameras.push_back (cam);
    m_CameraImage.data.image.format = Img::CF_RGB;
    m_CameraImage.data.image.width = cam->getWidth ();
    m_CameraImage.data.image.height = cam->getHeight ();
    m_CameraImage.data.image.raw_data.length (cam->getWidth () * cam->getHeight () * 3);
  }else{
    m_MultiCameraImages.data.image_seq.length (m_devIds.size ());
    m_MultiCameraImages.data.camera_set_id = 0;
    for (unsigned int i = 0; i < m_devIds.size (); i++)
      {
	std::cout << "** devId:" << m_devIds[i] << std::endl;
	v4l_capture *cam = new v4l_capture ();
	cam->init(m_width, m_height, m_devIds[i]);
	m_cameras.push_back (cam);
	m_MultiCameraImages.data.image_seq[i].image.format = Img::CF_RGB;
	m_MultiCameraImages.data.image_seq[i].image.width = cam->getWidth ();
	m_MultiCameraImages.data.image_seq[i].image.height = cam->getHeight ();
	m_MultiCameraImages.data.image_seq[i].image.raw_data.length (cam->getWidth () * cam->getHeight () * 3);
      }
  }

  return RTC::RTC_OK;
}

RTC::ReturnCode_t VideoCapture::onDeactivated(RTC::UniqueId ec_id)
{
  std::cout << m_profile.instance_name<< ": onDeactivated(" << ec_id << ")" << std::endl;
  for (unsigned int i=0; i< m_cameras.size(); i++){
      delete m_cameras[i];
  } 
  m_cameras.clear();
  return RTC::RTC_OK;
}

RTC::ReturnCode_t VideoCapture::onExecute(RTC::UniqueId ec_id)
{
  //std::cout << m_profile.instance_name<< ": onExecute(" << ec_id << ")" << std::endl;
  if (m_needToReactivate){
    if (onActivated(ec_id) == RTC::RTC_OK){
      m_needToReactivate = false;
    }
  }
  if (!capture()){
    std::cerr << m_profile.instance_name << ": failed to capture." << std::endl;
    onDeactivated(ec_id);
    m_needToReactivate = true;
    return RTC::RTC_OK;
  }

  double tNew = (double)(coil::gettimeofday());
  double dt = (double)(tNew - m_tOld);
  if (dt > 1.0/m_frameRate){
    m_tOld = tNew;
  }else{
    return RTC::RTC_OK;
  }

  if (m_mode == SLEEP) return RTC::RTC_OK;

  if (m_cameras.size() == 1){
    m_CameraImageOut.write();
  }else{
    m_MultiCameraImagesOut.write();
  }

  if (m_mode == ONESHOT) m_mode = SLEEP;

  return RTC::RTC_OK;
}

bool VideoCapture::capture()
{
  if (m_cameras.size() == 1){
    m_CameraImage.error_code = 0;
    uchar *imgFrom = m_cameras[0]->capture();
    if (!imgFrom) return false;
    memcpy(m_CameraImage.data.image.raw_data.get_buffer(), imgFrom,
	   m_CameraImage.data.image.raw_data.length() * sizeof(uchar));
  }else{
    m_MultiCameraImages.error_code = 0;
    for (unsigned int i = 0; i < m_cameras.size (); i++)
      {
	uchar *imgFrom = m_cameras[i]->capture();
        if (!imgFrom) return false;
	memcpy (m_MultiCameraImages.data.image_seq[i].image.raw_data.get_buffer(), imgFrom,
		m_MultiCameraImages.data.image_seq[i].image.raw_data.length() * sizeof (uchar));
      }
  }
  return true;
}

/*
RTC::ReturnCode_t VideoCapture::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t VideoCapture::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t VideoCapture::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t VideoCapture::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t VideoCapture::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

void VideoCapture::take_one_frame()
{
  m_mode = ONESHOT;
}

void VideoCapture::start_continuous()
{
  m_mode = CONTINUOUS;
}

void VideoCapture::stop_continuous()
{
  m_mode = SLEEP;
}


extern "C"
{

  void VideoCaptureInit(RTC::Manager* manager)
  {
    RTC::Properties profile(videocapture_spec);
    manager->registerFactory(profile,
                             RTC::Create<VideoCapture>,
                             RTC::Delete<VideoCapture>);
  }

};


