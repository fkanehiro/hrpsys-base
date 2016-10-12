// -*- C++ -*-
/*!
 * @file  Joystick2Velocity2D.cpp
 * @brief joystick output to velocity converter
 * $Date$
 *
 * $Id$
 */

#include <rtm/idl/BasicDataType.hh>
#include <rtm/idl/ExtendedDataTypes.hh>
#include "hrpsys/util/VectorConvert.h"
#include "Joystick2Velocity2D.h"

// Module specification
// <rtc-template block="module_spec">
static const char* joystick2velocity_spec[] =
  {
    "implementation_id", "Joystick2Velocity2D",
    "type_name",         "Joystick2Velocity2D",
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

Joystick2Velocity2D::Joystick2Velocity2D(RTC::Manager* manager)
  : RTC::DataFlowComponentBase(manager),
    // <rtc-template block="initializer">
    m_axesIn("axes", m_axes),
    m_velOut("vel", m_vel),
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

Joystick2Velocity2D::~Joystick2Velocity2D()
{
}



RTC::ReturnCode_t Joystick2Velocity2D::onInitialize()
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
  addOutPort("vel", m_velOut);

  m_axes.data.length(4);
  for (unsigned int i=0; i<m_axes.data.length(); i++){
      m_axes.data[i] = 0.0;
  }

  return RTC::RTC_OK;
}



/*
RTC::ReturnCode_t Joystick2Velocity2D::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t Joystick2Velocity2D::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t Joystick2Velocity2D::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

RTC::ReturnCode_t Joystick2Velocity2D::onActivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}

RTC::ReturnCode_t Joystick2Velocity2D::onDeactivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}

RTC::ReturnCode_t Joystick2Velocity2D::onExecute(RTC::UniqueId ec_id)
{
  if (m_debugLevel > 0){
      std::cout << m_profile.instance_name<< ": onExecute(" << ec_id << ")" 
                << std::endl;
  }

  if (m_axesIn.isNew()) m_axesIn.read();

  m_vel.data.vx = m_neutrals[0] + m_scales[0]*m_axes.data[m_axesIds[0]];
  m_vel.data.vy = m_neutrals[1] + m_scales[1]*m_axes.data[m_axesIds[1]];
  m_vel.data.va = m_neutrals[2] + m_scales[2]*m_axes.data[m_axesIds[2]];
  if (m_debugLevel > 0) {
      printf("velocity command: %5.2f %5.2f %5.2f", 
             m_vel.data.vx, m_vel.data.vy, m_vel.data.va);
  }
  m_velOut.write();

  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t Joystick2Velocity2D::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t Joystick2Velocity2D::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t Joystick2Velocity2D::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t Joystick2Velocity2D::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t Joystick2Velocity2D::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/



extern "C"
{

  void Joystick2Velocity2DInit(RTC::Manager* manager)
  {
    RTC::Properties profile(joystick2velocity_spec);
    manager->registerFactory(profile,
                             RTC::Create<Joystick2Velocity2D>,
                             RTC::Delete<Joystick2Velocity2D>);
  }

};


