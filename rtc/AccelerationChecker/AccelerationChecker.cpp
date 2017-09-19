// -*- C++ -*-
/*!
 * @file  AccelerationChecker.cpp
 * @brief joint acceleration checker
 * $Date$
 *
 * $Id$
 */

#include <math.h>
#include <iomanip>
#include <stdio.h>
#include "AccelerationChecker.h"
#include "hrpsys/idl/RobotHardwareService.hh"

// Module specification
// <rtc-template block="module_spec">
static const char* spec[] =
  {
    "implementation_id", "AccelerationChecker",
    "type_name",         "AccelerationChecker",
    "description",       "Joint acceleration checker",
    "version",           HRPSYS_PACKAGE_VERSION,
    "vendor",            "AIST",
    "category",          "example",
    "activity_type",     "DataFlowComponent",
    "max_instance",      "10",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    "conf.default.thd", "1000",
    "conf.default.print", "0",

    ""
  };
// </rtc-template>

AccelerationChecker::AccelerationChecker(RTC::Manager* manager)
  : RTC::DataFlowComponentBase(manager),
    // <rtc-template block="initializer">
    m_qIn("qIn", m_q),
    m_servoStateIn("servoState", m_servoState),
    m_qOut("qOut", m_q),
    // </rtc-template>
    dummy(0)
{
}

AccelerationChecker::~AccelerationChecker()
{
}



RTC::ReturnCode_t AccelerationChecker::onInitialize()
{
  //std::cout << m_profile.instance_name << ": onInitialize()" << std::endl;
  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  bindParameter("thd", m_thd, "1000");
  bindParameter("print", m_print, "0");
  
  // </rtc-template>

  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("qIn", m_qIn);
  addInPort("servoState", m_servoStateIn);

  // Set OutPort buffer
  addOutPort("qOut", m_qOut);
  
  // Set service provider to Ports
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  
  // </rtc-template>

  RTC::Properties& prop = getProperties();
  m_dt = 0;
  coil::stringTo(m_dt, prop["dt"].c_str());
  if (m_dt == 0){
      std::cerr << m_profile.instance_name << ": dt is not defined in the conf"
                << std::endl;
      return RTC::RTC_ERROR;
  }

  return RTC::RTC_OK;
}



/*
RTC::ReturnCode_t AccelerationChecker::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t AccelerationChecker::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t AccelerationChecker::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

RTC::ReturnCode_t AccelerationChecker::onActivated(RTC::UniqueId ec_id)
{
  std::cout << m_profile.instance_name<< ": onActivated(" << ec_id << ")" << std::endl;
  return RTC::RTC_OK;
}

RTC::ReturnCode_t AccelerationChecker::onDeactivated(RTC::UniqueId ec_id)
{
  std::cout << m_profile.instance_name<< ": onDeactivated(" << ec_id << ")" << std::endl;
  return RTC::RTC_OK;
}

RTC::ReturnCode_t AccelerationChecker::onExecute(RTC::UniqueId ec_id)
{
  //std::cout << m_profile.instance_name<< ": onExecute(" << ec_id << ")" << std::endl;

  if (m_qIn.isNew()){
    m_qIn.read();
    while(m_servoStateIn.isNew()) m_servoStateIn.read();
    if (!m_dq.data.length()){ // first time
        m_qOld.data.length(m_q.data.length());
        m_qOld = m_q;
        m_dqOld.data.length(m_q.data.length());
        m_ddqMax.data.length(m_q.data.length());
        for (unsigned int i=0; i<m_dqOld.data.length(); i++){
            m_dqOld.data[i] = 0.0;
            m_ddqMax.data[i] = 0.0;
        }
        m_dq.data.length(m_q.data.length());
    }
    for (unsigned int i=0; i<m_q.data.length(); i++){
        m_dq.data[i] = (m_q.data[i] - m_qOld.data[i])/m_dt;
        double ddq = (m_dq.data[i] - m_dqOld.data[i])/m_dt;
        bool servo = true;
        if (m_servoState.data.length()==m_q.data.length()){
            servo = m_servoState.data[i][0] & OpenHRP::RobotHardwareService::SERVO_STATE_MASK;
        }
        if (fabs(ddq) > m_ddqMax.data[i]) m_ddqMax.data[i] = fabs(ddq);
        if (servo && fabs(ddq) > m_thd){
            time_t now = time(NULL);
            struct tm *tm_now = localtime(&now);
            char *datetime = asctime(tm_now);
            datetime[strlen(datetime)-1] = '\0'; 
            std::cout << "[" 
                      << datetime
                      << "] Warning: too big joint acceleration for "
                      << i << "th joint(" << ddq << "[rad/m^2])" << std::endl;
        }
    }
    m_qOld = m_q;
    m_dqOld = m_dq;

    if (m_print){
        printf("jid: max acc[rad/m^2]\n");
        for (unsigned int i=0; i<m_ddqMax.data.length(); i++){
            printf("%2d: %8f\n", i, m_ddqMax.data[i]);
        }
        m_print = false;
    }

    m_qOut.write();
  }

  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t AccelerationChecker::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t AccelerationChecker::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t AccelerationChecker::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t AccelerationChecker::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t AccelerationChecker::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/



extern "C"
{

  void AccelerationCheckerInit(RTC::Manager* manager)
  {
    RTC::Properties profile(spec);
    manager->registerFactory(profile,
                             RTC::Create<AccelerationChecker>,
                             RTC::Delete<AccelerationChecker>);
  }

};


