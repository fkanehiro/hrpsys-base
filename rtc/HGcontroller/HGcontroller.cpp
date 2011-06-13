// -*- C++ -*-
/*!
 * @file  HGcontroller.cpp
 * @brief high gain joint controller component for OpenHRP
 * $Date$
 *
 * $Id$
 */

#include "HGcontroller.h"

// Module specification
// <rtc-template block="module_spec">
static const char* hgcontroller_spec[] =
  {
    "implementation_id", "HGcontroller",
    "type_name",         "HGcontroller",
    "description",       "high gain joint controller component for OpenHRP",
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

HGcontroller::HGcontroller(RTC::Manager* manager)
  : RTC::DataFlowComponentBase(manager),
    // <rtc-template block="initializer">
    m_qIn("qIn", m_q),
    m_qOut("qOut", m_q),
    m_dqOut("dqOut", m_dq),
    m_ddqOut("ddqOut", m_ddq),
    // </rtc-template>
    m_dt(0.005),
	dummy(0)
{
}

HGcontroller::~HGcontroller()
{
}



RTC::ReturnCode_t HGcontroller::onInitialize()
{
  std::cout << m_profile.instance_name << ": onInitialize()" << std::endl;

  RTC::Properties& ref = getProperties();
  coil::stringTo(m_dt, ref["dt"].c_str());

  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  
  // </rtc-template>

  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("qIn", m_qIn);

  // Set OutPort buffer
  addOutPort("qOut", m_qOut);
  addOutPort("dqOut", m_dqOut);
  addOutPort("ddqOut", m_ddqOut);
  
  // Set service provider to Ports
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  
  // </rtc-template>

  return RTC::RTC_OK;
}



/*
RTC::ReturnCode_t HGcontroller::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t HGcontroller::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t HGcontroller::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

RTC::ReturnCode_t HGcontroller::onActivated(RTC::UniqueId ec_id)
{
  std::cout << m_profile.instance_name<< ": onActivated(" << ec_id << ")" << std::endl;
  return RTC::RTC_OK;
}

RTC::ReturnCode_t HGcontroller::onDeactivated(RTC::UniqueId ec_id)
{
  std::cout << m_profile.instance_name<< ": onDeactivated(" << ec_id << ")" << std::endl;
  return RTC::RTC_OK;
}

RTC::ReturnCode_t HGcontroller::onExecute(RTC::UniqueId ec_id)
{
  //std::cout << m_profile.instance_name<< ": onExecute(" << ec_id << ")" << std::endl;

  if (m_qIn.isNew()) m_qIn.read();

  if (m_q.data.length()){
      if (m_dq.data.length() == 0){
          // initialize m_dq, m_ddq
          m_dq.data.length(m_q.data.length());
          m_ddq.data.length(m_q.data.length());
          for (unsigned int i=0; i<m_q.data.length(); i++){
              m_dq.data[i] = m_ddq.data[i] = 0.0;
          }
      }else{
          // compute m_dq, m_ddq
          for (unsigned int i=0; i<m_q.data.length(); i++){
              m_dq.data[i] = (m_q.data[i] - m_qOld.data[i])/m_dt;
              m_ddq.data[i] = (m_dq.data[i] - m_dqOld.data[i])/m_dt;
          }
      }

      m_qOut.write();
      m_dqOut.write();
      m_ddqOut.write();

      m_qOld = m_q;
      m_dqOld = m_dq;
  }

  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t HGcontroller::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t HGcontroller::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t HGcontroller::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t HGcontroller::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t HGcontroller::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/



extern "C"
{

  void HGcontrollerInit(RTC::Manager* manager)
  {
    RTC::Properties profile(hgcontroller_spec);
    manager->registerFactory(profile,
                             RTC::Create<HGcontroller>,
                             RTC::Delete<HGcontroller>);
  }

};


