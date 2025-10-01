// -*- C++ -*-
/*!
 * @file  SoftwareEmergency.cpp
 * @brief whole-body inverse kinematics based motion generator
 * $Date$
 *
 * $Id$
 */

#include <rtm/CorbaNaming.h>
#include "hrpsys/util/VectorConvert.h"
#include "hrpsys/idl/RobotHardwareService.hh"
#include "SoftwareEmergency.h"
#include "SoftwareEmergencyUtil.h"

// Module specification
// <rtc-template block="module_spec">
static const char* spec[] =
  {
    "implementation_id", "SoftwareEmergency",
    "type_name",         "SoftwareEmergency",
    "description",       "",
    "version",           "1.0",
    "vendor",            "AIST",
    "category",          "example",
    "activity_type",     "DataFlowComponent",
    "max_instance",      "10",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    "conf.default.servoErrorLimit", "0.18",
    "conf.default.gain", "1.0",

    ""
  };
// </rtc-template>

SoftwareEmergency::SoftwareEmergency(RTC::Manager* manager)
  : RTC::DataFlowComponentBase(manager),
    // <rtc-template block="initializer">
    m_qIn("qIn", m_q),
    m_qRefIn("qRefIn", m_qRef),
    m_servoStateIn("servoState", m_servoState),
    m_qRefOut("qRefOut", m_qRefMod),
    m_emergencySignalOut("emergencySignal", m_emergencySignal),
    m_buttonSignalOut("buttonSignal", m_buttonSignal),
    m_SoftwareEmergencyServicePort("SoftwareEmergencyService"),
    m_SoftwareEmergencyService(this),
    // </rtc-template>
    m_mode(::OpenHRP::SoftwareEmergencyService::THROUGH),
    m_check(false),
    dummy(0)
{
    m_button.resize(MAX_USBIO2_NUM-1, false);
}

SoftwareEmergency::~SoftwareEmergency()
{
}



RTC::ReturnCode_t SoftwareEmergency::onInitialize()
{
  //std::cout << m_profile.instance_name << ": onInitialize()" << std::endl;
  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  bindParameter("servoErrorLimit", m_servoErrorLimit, "0.15");
  bindParameter("gain", m_gain, "1.0");

  //bindParameter("mode", m_mode, "through");
  // </rtc-template>

  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("qIn", m_qIn);
  addInPort("qRefIn", m_qRefIn);
  addInPort("servoState", m_servoStateIn);
  
  // Set OutPort buffer
  addOutPort("qRefOut", m_qRefOut);
  addOutPort("emergencySignal", m_emergencySignalOut);
  addOutPort("buttonSignal", m_buttonSignalOut);
  
  // Set service provider to Ports
  m_SoftwareEmergencyServicePort.registerProvider("service0", "SoftwareEmergencyService", m_SoftwareEmergencyService);
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  addPort(m_SoftwareEmergencyServicePort);
  
  // </rtc-template>

  RTC::Properties& prop = getProperties();
  coil::stringTo(m_dt, prop["dt"].c_str());
  coil::stringTo(m_servoErrorLimits, prop["servo_error_limit"].c_str());
  if (m_servoErrorLimits.size()){
    for (unsigned int i=0; i<m_servoErrorLimits.size(); i++){
      m_servoErrorLimits[i] *= M_PI/180;
    }
  }

  m_emergencySignal.data = 100;
  return RTC::RTC_OK;
}



RTC::ReturnCode_t SoftwareEmergency::onFinalize()
{
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t SoftwareEmergency::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t SoftwareEmergency::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

RTC::ReturnCode_t SoftwareEmergency::onActivated(RTC::UniqueId ec_id)
{
  std::cout << m_profile.instance_name<< ": onActivated(" << ec_id << ")" << std::endl;

  return RTC::RTC_OK;
}

RTC::ReturnCode_t SoftwareEmergency::onDeactivated(RTC::UniqueId ec_id)
{
  std::cout << m_profile.instance_name<< ": onDeactivated(" << ec_id << ")" << std::endl;

  return RTC::RTC_OK;
}

void SoftwareEmergency::checkServoError()
{
  if (m_q.data.length() && m_q.data.length() == m_qRef.data.length()){
    if (!m_servoErrorLimits.size()){
      m_servoErrorLimits.resize(m_q.data.length());
      for (unsigned int i=0;i<m_servoErrorLimits.size(); i++){
        m_servoErrorLimits[i] = m_servoErrorLimit;
      }
    }
    
    for (unsigned int i=0; i<m_q.data.length(); i++){
      double thd = m_servoErrorLimits[i];
      if (isServoOn(i) && thd && fabs(m_q.data[i] - m_qRef.data[i]) > thd){
        std::cout << "\033[31m" << m_profile.instance_name
                  << ": servo error limit over: joint = " << i
                  << ", qRef = " << m_qRef.data[i]*180/M_PI
                  << "[deg], q = " << m_q.data[i]*180/M_PI
                  << "[deg], limit = " << thd*180/M_PI << "\033[0m"
                  << std::endl;
	switchModeTo(::OpenHRP::SoftwareEmergencyService::RELAX);
        break;
      }
    }
  }
}

RTC::ReturnCode_t SoftwareEmergency::onExecute(RTC::UniqueId ec_id)
{
  //std::cout << m_profile.instance_name<< ": onExecute(" << ec_id << ")" << std::endl;
  while(m_servoStateIn.isNew()) m_servoStateIn.read();

  // go back to THROUGH mode if servo is off
  if (!isServoOn()){
    switchModeTo(::OpenHRP::SoftwareEmergencyService::THROUGH);
  }

  bool newMes=false;
  while(m_qIn.isNew()) {
    newMes = true;
    m_qIn.read();
  }

  bool newCmd=false;
  while(m_qRefIn.isNew()) {
    newCmd = true;
    m_qRefIn.read();
  }

  if (m_mode == ::OpenHRP::SoftwareEmergencyService::THROUGH && m_check) {
    checkServoError();
    m_check = false;
  }

  switch(m_mode){
  case ::OpenHRP::SoftwareEmergencyService::THROUGH:
    if (newCmd) m_check = true;
    m_qRefMod = m_qRef;
    break;
  case ::OpenHRP::SoftwareEmergencyService::FREEZE:
    break;
  case ::OpenHRP::SoftwareEmergencyService::RELAX:
    if (newMes) newCmd = true;
#define VLIMIT 1.5 //[rad/s]
    for (unsigned int i=0; i<m_qRefMod.data.length(); i++){
      double maxdq = VLIMIT*m_dt;
      double dq = m_q.data[i] - m_qRefMod.data[i];
      if (dq > maxdq){
	dq = maxdq;
      }else if(dq < -maxdq){
	dq = -maxdq;
      }
      m_qRefMod.data[i] += m_gain*dq;
    }
    break;
  default:
    break;
  }

  if (newCmd) m_qRefOut.write();
  
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t SoftwareEmergency::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t SoftwareEmergency::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t SoftwareEmergency::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t SoftwareEmergency::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t SoftwareEmergency::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

bool SoftwareEmergency::isServoOn()
{
  bool servo = !m_servoState.data.length();
  for (unsigned int i=0; i<m_servoState.data.length(); i++){
    if (m_servoState.data[i][0] & OpenHRP::RobotHardwareService::SERVO_STATE_MASK){
      servo = true;
      break;
    }
  }
  return servo;
}

bool SoftwareEmergency::isServoOn(int i)
{
  return m_servoState.data.length() ? m_servoState.data[i][0] & OpenHRP::RobotHardwareService::SERVO_STATE_MASK : true;
}

bool SoftwareEmergency::switchModeTo(::OpenHRP::SoftwareEmergencyService::mode m)
{
  if (m_mode == m) return true;

  if (m_mode == ::OpenHRP::SoftwareEmergencyService::THROUGH && m == ::OpenHRP::SoftwareEmergencyService::RELAX){
    m_mode = m;
    std::cout << "\033[31m" << m_profile.instance_name
	      << ": switch mode to RELAX" << "\033[0m"
	      << std::endl;
    m_emergencySignalOut.write();
    return true;
  }else if(m_mode == ::OpenHRP::SoftwareEmergencyService::RELAX && m == ::OpenHRP::SoftwareEmergencyService::THROUGH){
    if (!isServoOn()){
      m_mode = m;
      return true;
    }else{
      std::cout << m_profile.instance_name << ": servo off first!" << std::endl;
    }
  }else{
    std::cout << m_profile.instance_name << ": switching mode from " << m_mode
	      << " to " << m << " is not supported" << std::endl;
  }
  return false;
}

bool SoftwareEmergency::setButtonStatus(CORBA::Long id, CORBA::Boolean on)
{
  if (m_button[id] == on) return true;
  if (id >= m_button.size()) return true;
  
  if ( !m_button[id] && on ){
    m_button[id] = on;
    std::cout << "\033[33m" << m_profile.instance_name
	      << ": pressed button #" << id << "\033[0m"
	      << std::endl;
    m_buttonSignal.data = 1 + (id << 1);
    m_buttonSignalOut.write();
    return true;
  }else if( m_button[id] && !on ){
    m_button[id] = on;
    std::cout << "\033[33m" << m_profile.instance_name
	      << ": released button #" << id << "\033[0m"
	      << std::endl;
    m_buttonSignal.data = 0 + (id << 1);
    m_buttonSignalOut.write();
    return true;
  }else{
    std::cout << m_profile.instance_name << ": changing button status from " << m_button[id]
	      << " to " << on << " is not supported" << std::endl;
  }
  return false;
}

extern "C"
{

  void SoftwareEmergencyInit(RTC::Manager* manager)
  {
    RTC::Properties profile(spec);
    manager->registerFactory(profile,
                             RTC::Create<SoftwareEmergency>,
                             RTC::Delete<SoftwareEmergency>);
  }

};


