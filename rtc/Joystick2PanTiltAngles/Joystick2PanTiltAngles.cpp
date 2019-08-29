// -*- C++ -*-
/*!
 * @file  Joystick2PanTiltAngles.cpp
 * @brief joystick output to velocity converter
 * $Date$
 *
 * $Id$
 */

#include "hrpsys/util/VectorConvert.h"
#include "Joystick2PanTiltAngles.h"

// Module specification
// <rtc-template block="module_spec">
static const char* joystick2velocity_spec[] =
  {
    "implementation_id", "Joystick2PanTiltAngles",
    "type_name",         "Joystick2PanTiltAngles",
    "description",       "joystick output to velocity converter",
    "version",           HRPSYS_PACKAGE_VERSION,
    "vendor",            "AIST",
    "category",          "example",
    "activity_type",     "DataFlowComponent",
    "max_instance",      "10",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    "conf.default.debugLevel", "0",
    "conf.default.axesIds", "0,1,2",
    "conf.default.scales", "1.0,1.0,1.0",
    "conf.default.neutrals", "0.0,0.0,0.0",
    ""
  };
// </rtc-template>

Joystick2PanTiltAngles::Joystick2PanTiltAngles(RTC::Manager* manager)
  : RTC::DataFlowComponentBase(manager),
    // <rtc-template block="initializer">
    m_axesIn("axes", m_axes),
    m_anglesOut("angles", m_angles),
    // </rtc-template>
    dummy(0),
    m_debugLevel(0)
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers

  // Set OutPort buffer
  
  // Set service provider to Ports
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  
  // </rtc-template>
}

Joystick2PanTiltAngles::~Joystick2PanTiltAngles()
{
}



RTC::ReturnCode_t Joystick2PanTiltAngles::onInitialize()
{
  //std::cout << m_profile.instance_name << ": onInitialize()" << std::endl;
  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  bindParameter("debugLevel", m_debugLevel, "0");
  bindParameter("axesIds", m_axesIds, "0,1,2");
  bindParameter("scales", m_scales, "1.0,1.0,1.0");
  bindParameter("neutrals", m_neutrals, "0.0,0.0,0.0");

  // </rtc-template>
  addInPort("axes", m_axesIn);
  addOutPort("vel", m_anglesOut);

  m_axes.data.length(4);
  for (unsigned int i=0; i<m_axes.data.length(); i++){
      m_axes.data[i] = 0.0;
  }

  return RTC::RTC_OK;
}



/*
RTC::ReturnCode_t Joystick2PanTiltAngles::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t Joystick2PanTiltAngles::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t Joystick2PanTiltAngles::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

RTC::ReturnCode_t Joystick2PanTiltAngles::onActivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}

RTC::ReturnCode_t Joystick2PanTiltAngles::onDeactivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}

RTC::ReturnCode_t Joystick2PanTiltAngles::onExecute(RTC::UniqueId ec_id)
{
  if (m_debugLevel > 0){
      std::cout << m_profile.instance_name<< ": onExecute(" << ec_id << ")" 
                << std::endl;
  }

  if (m_axesIn.isNew()) m_axesIn.read();

  m_angles.pan  = m_neutrals[0] + m_scales[0]*m_axes.data[m_axesIds[0]];
  m_angles.tilt = m_neutrals[1] + m_scales[1]*m_axes.data[m_axesIds[1]];
  if (m_debugLevel > 0) {
      printf("pan/tilt command: %5.2f %5.2f", 
             m_angles.pan, m_angles.tilt);
  }
  m_anglesOut.write();

  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t Joystick2PanTiltAngles::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t Joystick2PanTiltAngles::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t Joystick2PanTiltAngles::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t Joystick2PanTiltAngles::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t Joystick2PanTiltAngles::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/



extern "C"
{

  void Joystick2PanTiltAnglesInit(RTC::Manager* manager)
  {
    RTC::Properties profile(joystick2velocity_spec);
    manager->registerFactory(profile,
                             RTC::Create<Joystick2PanTiltAngles>,
                             RTC::Delete<Joystick2PanTiltAngles>);
  }

};


