// -*- C++ -*-
/*!
 * @file  SampleComponent.cpp
 * @brief null component
 * $Date$
 *
 * $Id$
 */

#include "SampleComponent.h"

// Module specification
// <rtc-template block="module_spec">
static const char* nullcomponent_spec[] =
  {
    "implementation_id", "SampleComponent",
    "type_name",         "SampleComponent",
    "description",       "null component",
    "version",           HRPSYS_PACKAGE_VERSION,
    "vendor",            "AIST",
    "category",          "example",
    "activity_type",     "DataFlowComponent",
    "max_instance",      "10",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    "conf.default.debugLevel", "0",
    ""
  };
// </rtc-template>

SampleComponent::SampleComponent(RTC::Manager* manager)
  : RTC::DataFlowComponentBase(manager),
    // <rtc-template block="initializer">
    m_qCurrentIn("qCurrent", m_qCurrent),
    m_qOut("q", m_q),
    m_SampleComponentPort("SampleComponentService"),
    loop(0),
    offset(0),
    m_debugLevel(0),
    dummy(0)
{
  m_service0.sample(this);
  std::cerr << "SampleComponent::SampleComponent()" << std::endl;
}

SampleComponent::~SampleComponent()
{
  std::cerr << "SampleComponent::~SampleComponent()" << std::endl;
}



RTC::ReturnCode_t SampleComponent::onInitialize()
{
  std::cerr << m_profile.instance_name << ": onInitialize()" << std::endl;
  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  
  // </rtc-template>

  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("qCurrent", m_qCurrentIn);

  // Set OutPort buffer
  addOutPort("q", m_qOut);
  
  // Set service provider to Ports
  m_SampleComponentPort.registerProvider("service0", "SampleComponentService", m_service0);
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  addPort(m_SampleComponentPort);
  
  // </rtc-template>

  return RTC::RTC_OK;
}



/*
RTC::ReturnCode_t SampleComponent::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t SampleComponent::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t SampleComponent::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

RTC::ReturnCode_t SampleComponent::onActivated(RTC::UniqueId ec_id)
{
  std::cerr << m_profile.instance_name<< ": onActivated(" << ec_id << ")" << std::endl;
  return RTC::RTC_OK;
}

RTC::ReturnCode_t SampleComponent::onDeactivated(RTC::UniqueId ec_id)
{
  std::cerr << m_profile.instance_name<< ": onDeactivated(" << ec_id << ")" << std::endl;
  return RTC::RTC_OK;
}

bool SampleComponent::resetOffset(double dir) {
  loop = 0;
  offset = dir;
  return true;
}

RTC::ReturnCode_t SampleComponent::onExecute(RTC::UniqueId ec_id)
{
 //std::cerr << m_profile.instance_name<< ": onExecute(" << ec_id << ")" << std::endl;
  loop ++;
  static double output = 0;
  if (m_qCurrentIn.isNew()) {
    m_qCurrentIn.read();
  }

  if ( m_qCurrent.data.length() != m_q.data.length() ) {
      m_q.data.length(m_qCurrent.data.length());
  }
  for ( unsigned int i = 0; i < m_qCurrent.data.length(); i++ ){
      m_q.data[i] = m_qCurrent.data[i];
  }
  if ( loop < 1000 ) {
      output += offset;
  }
  if ( m_q.data.length() > 9 ) {
      m_q.data[9] = output;
  }
  m_qOut.write();

  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t SampleComponent::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t SampleComponent::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t SampleComponent::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t SampleComponent::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t SampleComponent::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/



extern "C"
{

  void SampleComponentInit(RTC::Manager* manager)
  {
    RTC::Properties profile(nullcomponent_spec);
    manager->registerFactory(profile,
                             RTC::Create<SampleComponent>,
                             RTC::Delete<SampleComponent>);
  }

};


