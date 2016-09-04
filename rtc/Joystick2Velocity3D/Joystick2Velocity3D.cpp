// -*- C++ -*-
/*!
 * @file  Joystick2Velocity3D.cpp
 * @brief joystick output to velocity converter
 * $Date$
 *
 * $Id$
 */

#include <rtm/idl/BasicDataType.hh>
#include <rtm/idl/ExtendedDataTypes.hh>
#include "hrpsys/util/VectorConvert.h"
#include "Joystick2Velocity3D.h"

// Module specification
// <rtc-template block="module_spec">
static const char* joystick2velocity_spec[] =
  {
    "implementation_id", "Joystick2Velocity3D",
    "type_name",         "Joystick2Velocity3D",
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
    "conf.default.scalesTranslation", "1.0,1.0,1.0",
    "conf.default.scalesRotation", "1.0,1.0,1.0",
    "conf.default.rotateModeButton", "9",
    ""
  };
// </rtc-template>

Joystick2Velocity3D::Joystick2Velocity3D(RTC::Manager* manager)
  : RTC::DataFlowComponentBase(manager),
    // <rtc-template block="initializer">
    m_axesIn("axes", m_axes),
    m_buttonsIn("buttons", m_buttons),
    m_velOut("vel", m_vel),
    m_mirroredVelOut("mirroredVel", m_mirroredVel),
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

Joystick2Velocity3D::~Joystick2Velocity3D()
{
}



RTC::ReturnCode_t Joystick2Velocity3D::onInitialize()
{
  //std::cout << m_profile.instance_name << ": onInitialize()" << std::endl;
  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  bindParameter("debugLevel", m_debugLevel, "0");
  bindParameter("axesIds", m_axesIds, "0,1,2");
  bindParameter("scalesTranslation", m_scalesTranslation, "1.0,1.0,1.0");
  bindParameter("scalesRotation", m_scalesRotation, "1.0,1.0,1.0");
  bindParameter("rotateModeButton", m_rotateModeButton, "9");

  // </rtc-template>
  addInPort("axes", m_axesIn);
  addInPort("buttons", m_buttonsIn);
  addOutPort("vel", m_velOut);
  addOutPort("mirroredVel", m_mirroredVelOut);

  m_vel.data.vx = m_vel.data.vy = m_vel.data.vz = 0;
  m_vel.data.vr = m_vel.data.vp = m_vel.data.va = 0;
  m_mirroredVel.data.vx = m_mirroredVel.data.vy = m_mirroredVel.data.vz = 0;
  m_mirroredVel.data.vr = m_mirroredVel.data.vp = m_mirroredVel.data.va = 0;
  m_axes.data.length(4);
  for (unsigned int i=0; i<m_axes.data.length(); i++){
      m_axes.data[i] = 0.0;
  }

  return RTC::RTC_OK;
}



/*
RTC::ReturnCode_t Joystick2Velocity3D::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t Joystick2Velocity3D::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t Joystick2Velocity3D::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

RTC::ReturnCode_t Joystick2Velocity3D::onActivated(RTC::UniqueId ec_id)
{
  std::cout << "Joystick2Velocity3D::onActivated(" << ec_id << ")" << std::endl;
  return RTC::RTC_OK;
}

RTC::ReturnCode_t Joystick2Velocity3D::onDeactivated(RTC::UniqueId ec_id)
{
  std::cout << "Joystick2Velocity3D::onDeactivated(" << ec_id << ")" << std::endl;
  return RTC::RTC_OK;
}

RTC::ReturnCode_t Joystick2Velocity3D::onExecute(RTC::UniqueId ec_id)
{
  if (m_debugLevel > 0){
      std::cout << m_profile.instance_name<< ": onExecute(" << ec_id << ")" 
                << std::endl;
  }

  if (m_axesIn.isNew()) m_axesIn.read();
  if (m_buttonsIn.isNew()) m_buttonsIn.read();
  
  bool isPushed = false;
  for( unsigned int i = 0 ; i < m_buttons.data.length() ; i++ )
    isPushed |= m_buttons.data[i];
  
  if (m_buttons.data[m_rotateModeButton]){
      m_vel.data.vx = m_vel.data.vy = m_vel.data.vz = 0.0;
      m_vel.data.vr = -m_scalesRotation[1]*m_axes.data[m_axesIds[1]];
      m_vel.data.vp =  m_scalesRotation[0]*m_axes.data[m_axesIds[0]];
      m_vel.data.va =  m_scalesRotation[2]*m_axes.data[m_axesIds[2]];
      //
      m_mirroredVel.data.vx = m_mirroredVel.data.vy = m_mirroredVel.data.vz = 0.0;
      m_mirroredVel.data.vr = -m_vel.data.vr;
      m_mirroredVel.data.vp =  m_vel.data.vp;
      m_mirroredVel.data.va = -m_vel.data.va;
  }else if( !isPushed ){
      m_vel.data.vx = m_scalesTranslation[0]*m_axes.data[m_axesIds[0]];
      m_vel.data.vy = m_scalesTranslation[1]*m_axes.data[m_axesIds[1]];
      m_vel.data.vz = m_scalesTranslation[2]*m_axes.data[m_axesIds[2]];
      m_vel.data.vr = m_vel.data.vp = m_vel.data.va = 0.0;
      //
      m_mirroredVel.data.vx =  m_vel.data.vx;
      m_mirroredVel.data.vy = -m_vel.data.vy;
      m_mirroredVel.data.vz =  m_vel.data.vz;
      m_mirroredVel.data.vr = m_mirroredVel.data.vp = m_mirroredVel.data.va = 0.0;
  }
  if (m_debugLevel > 0) {
      printf("velocity command: %5.2f %5.2f %5.2f %5.2f %5.2f %5.2f\n", 
             m_vel.data.vx, m_vel.data.vy, m_vel.data.vz,
             m_vel.data.vr, m_vel.data.vp, m_vel.data.va);
  }
  m_velOut.write();
  m_mirroredVelOut.write();

  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t Joystick2Velocity3D::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t Joystick2Velocity3D::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t Joystick2Velocity3D::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t Joystick2Velocity3D::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t Joystick2Velocity3D::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/



extern "C"
{

  void Joystick2Velocity3DInit(RTC::Manager* manager)
  {
    RTC::Properties profile(joystick2velocity_spec);
    manager->registerFactory(profile,
                             RTC::Create<Joystick2Velocity3D>,
                             RTC::Delete<Joystick2Velocity3D>);
  }

};


