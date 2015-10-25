// -*- coding:utf-8-unix; mode: c++; indent-tabs-mode: nil; c-basic-offset: 2; -*-
/*!
 * @file  CaptureController.cpp
 * @brief capture controller
 * $Date$
 *
 * $Id$
 */

#include "CaptureController.h"

// Module specification
// <rtc-template block="module_spec">
static const char* capturecontroller_spec[] =
  {
    "implementation_id", "CaptureController",
    "type_name",         "CaptureController",
    "description",       "capture controller",
    "version",           HRPSYS_PACKAGE_VERSION,
    "vendor",            "AIST",
    "category",          "example",
    "activity_type",     "DataFlowComponent",
    "max_instance",      "10",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    "conf.default.frameRate",  "1",
    "conf.default.initialMode", "sleep",

    ""
  };
// </rtc-template>

CaptureController::CaptureController(RTC::Manager* manager)
  : RTC::DataFlowComponentBase(manager),
    // <rtc-template block="initializer">
    m_imageIn("imageIn", m_image),
    m_imageOut("imageOut", m_image),
    m_CameraCaptureServicePort("CameraCaptureService"),
    m_CameraCaptureService(this),
    // </rtc-template>
    dummy(0)
{
}

CaptureController::~CaptureController()
{
}



RTC::ReturnCode_t CaptureController::onInitialize()
{
  std::cout << m_profile.instance_name << ": onInitialize()" << std::endl;
  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  bindParameter("frameRate", m_frameRate, "1");
  bindParameter("initialMode", m_initialMode, "sleep");
  
  // </rtc-template>

  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("imageIn", m_imageIn);

  // Set OutPort buffer
  addOutPort("imageOut", m_imageOut);
  
  // Set service provider to Ports
  m_CameraCaptureServicePort.registerProvider("service0", "CameraCaptureService", m_CameraCaptureService);
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  addPort(m_CameraCaptureServicePort);
  
  // </rtc-template>

  RTC::Properties& prop = getProperties();

  return RTC::RTC_OK;
}



/*
RTC::ReturnCode_t CaptureController::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t CaptureController::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t CaptureController::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

RTC::ReturnCode_t CaptureController::onActivated(RTC::UniqueId ec_id)
{
  std::cout << m_profile.instance_name<< ": onActivated(" << ec_id << ")" << std::endl;
  m_tOld = (double)(coil::gettimeofday());
  if (m_initialMode == "continuous"){
    m_mode = CONTINUOUS;
  }else if(m_initialMode == "oneshot"){
    m_mode = ONESHOT;
  }else{
    m_mode = SLEEP;
  }
  return RTC::RTC_OK;
}

RTC::ReturnCode_t CaptureController::onDeactivated(RTC::UniqueId ec_id)
{
  std::cout << m_profile.instance_name<< ": onDeactivated(" << ec_id << ")" << std::endl;
  return RTC::RTC_OK;
}

RTC::ReturnCode_t CaptureController::onExecute(RTC::UniqueId ec_id)
{
  //std::cout << m_profile.instance_name<< ": onExecute(" << ec_id << ")" << m_data.data << std::endl;
  double tNew = (double)(coil::gettimeofday());
  double dt = (double)(tNew - m_tOld);
  if (m_mode != SLEEP && dt > 1.0/m_frameRate && m_imageIn.isNew()){
    m_imageIn.read();
    m_imageOut.write();
    if (m_mode == ONESHOT) m_mode = SLEEP;
    m_tOld = tNew;
  }
  
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t CaptureController::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t CaptureController::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t CaptureController::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t CaptureController::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t CaptureController::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


void CaptureController::take_one_frame()
{
  m_mode = ONESHOT;
}

void CaptureController::start_continuous()
{
  m_mode = CONTINUOUS;
}

void CaptureController::stop_continuous()
{
  m_mode = SLEEP;
}

extern "C"
{

  void CaptureControllerInit(RTC::Manager* manager)
  {
    RTC::Properties profile(capturecontroller_spec);
    manager->registerFactory(profile,
                             RTC::Create<CaptureController>,
                             RTC::Delete<CaptureController>);
  }

};


