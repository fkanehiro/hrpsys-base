// -*- C++ -*-
/*!
 * @file  RangeNoiseMixer.cpp
 * @brief plane remover
 * $Date$
 *
 * $Id$
 */

#include "RangeNoiseMixer.h"

// Module specification
// <rtc-template block="module_spec">
static const char* spec[] =
  {
    "implementation_id", "RangeNoiseMixer",
    "type_name",         "RangeNoiseMixer",
    "description",       "noise mixer for range data",
    "version",           HRPSYS_PACKAGE_VERSION,
    "vendor",            "AIST",
    "category",          "example",
    "activity_type",     "DataFlowComponent",
    "max_instance",      "10",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    "conf.default.maxDist", "0.02",

    ""
  };
// </rtc-template>

RangeNoiseMixer::RangeNoiseMixer(RTC::Manager* manager)
  : RTC::DataFlowComponentBase(manager),
    // <rtc-template block="initializer">
    m_rangeIn("original", m_range),
    m_rangeOut("mixed", m_range),
    // </rtc-template>
    dummy(0)
{
}

RangeNoiseMixer::~RangeNoiseMixer()
{
}



RTC::ReturnCode_t RangeNoiseMixer::onInitialize()
{
  //std::cout << m_profile.instance_name << ": onInitialize()" << std::endl;
  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  bindParameter("maxDist", m_maxDist, "0.02");
  
  // </rtc-template>

  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("original", m_rangeIn);

  // Set OutPort buffer
  addOutPort("mixed", m_rangeOut);
  
  // Set service provider to Ports
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  
  // </rtc-template>

  RTC::Properties& prop = getProperties();

  return RTC::RTC_OK;
}



/*
RTC::ReturnCode_t RangeNoiseMixer::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t RangeNoiseMixer::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t RangeNoiseMixer::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

RTC::ReturnCode_t RangeNoiseMixer::onActivated(RTC::UniqueId ec_id)
{
  std::cout << m_profile.instance_name<< ": onActivated(" << ec_id << ")" << std::endl;
  return RTC::RTC_OK;
}

RTC::ReturnCode_t RangeNoiseMixer::onDeactivated(RTC::UniqueId ec_id)
{
  std::cout << m_profile.instance_name<< ": onDeactivated(" << ec_id << ")" << std::endl;
  return RTC::RTC_OK;
}

RTC::ReturnCode_t RangeNoiseMixer::onExecute(RTC::UniqueId ec_id)
{
  //std::cout << m_profile.instance_name<< ": onExecute(" << ec_id << ")" << std::endl;

  if (m_rangeIn.isNew()){
    m_rangeIn.read();

    for (unsigned int i=0; i<m_range.ranges.length(); i++){
      if (m_range.ranges[i] != 0){
	m_range.ranges[i] += m_maxDist*((2.0*rand())/RAND_MAX-1.0);
      }
    }

    m_rangeOut.write();
  }

  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t RangeNoiseMixer::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t RangeNoiseMixer::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t RangeNoiseMixer::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t RangeNoiseMixer::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t RangeNoiseMixer::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/



extern "C"
{

  void RangeNoiseMixerInit(RTC::Manager* manager)
  {
    RTC::Properties profile(spec);
    manager->registerFactory(profile,
                             RTC::Create<RangeNoiseMixer>,
                             RTC::Delete<RangeNoiseMixer>);
  }

};


