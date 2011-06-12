// -*- C++ -*-
/*!
 * @file  WavPlayeer.cpp
 * @brief wave file player
 * $Date$
 *
 * $Id$
 */

#include "WavPlayer.h"

// Module specification
// <rtc-template block="module_spec">
static const char* wavplayer_spec[] =
  {
    "implementation_id", "WavPlayer",
    "type_name",         "WavPlayer",
    "description",       "wave file player",
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

WavPlayer::WavPlayer(RTC::Manager* manager)
  : RTC::DataFlowComponentBase(manager),
    // <rtc-template block="initializer">
    m_WavPlayerServicePort("WavPlayerService"),
    // </rtc-template>
	dummy(0)
{
}

WavPlayer::~WavPlayer()
{
}



RTC::ReturnCode_t WavPlayer::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  
  // Set OutPort buffer
  
  // Set service provider to Ports
  m_WavPlayerServicePort.registerProvider("service0", "WavPlayerService", m_service0);
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  addPort(m_WavPlayerServicePort);
  
  // </rtc-template>

  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  
  // </rtc-template>

  return RTC::RTC_OK;
}



/*
RTC::ReturnCode_t WavPlayer::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t WavPlayer::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t WavPlayer::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t WavPlayer::onActivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t WavPlayer::onDeactivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t WavPlayer::onExecute(RTC::UniqueId ec_id)
{
    return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t WavPlayer::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t WavPlayer::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t WavPlayer::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t WavPlayer::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t WavPlayer::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/



extern "C"
{

  void WavPlayerInit(RTC::Manager* manager)
  {
    RTC::Properties profile(wavplayer_spec);
    manager->registerFactory(profile,
                             RTC::Create<WavPlayer>,
                             RTC::Delete<WavPlayer>);
  }

};


