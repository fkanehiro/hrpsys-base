// -*- C++ -*-
/*!
 * @file  Beeper.cpp
 * @brief Beeper component
 * $Date$
 *
 * $Id$
 */

#include "Beeper.h"
#include <rtm/CorbaNaming.h>

// Variables for beep_thread
int beep_length = 0;
int beep_freq = 1000;
bool beep_start = false;
pthread_mutex_t beep_mutex;  // Mutex

// Module specification
// <rtc-template block="module_spec">
static const char* beeper_spec[] =
  {
    "implementation_id", "Beeper",
    "type_name",         "Beeper",
    "description",       "beeper",
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

void* call_beep (void* args)
{
  init_beep();
  while (1) {
    usleep(1000);
    pthread_mutex_lock( &beep_mutex );
    if (beep_start) {
      start_beep(beep_freq, beep_length);
    } else {
      stop_beep();
    }
    pthread_mutex_unlock( &beep_mutex );
  }
  quit_beep();
}

Beeper::Beeper(RTC::Manager* manager)
  : RTC::DataFlowComponentBase(manager),
    // <rtc-template block="initializer">
    m_dt(0.002),
    m_beepCommandIn("beepCommand", m_beepCommand),
    m_debugLevel(0)
{
  pthread_create(&beep_thread, NULL, call_beep, (void*)NULL);
  pthread_mutex_init( &beep_mutex, NULL );
}

Beeper::~Beeper()
{
  pthread_join(beep_thread, NULL );
}

RTC::ReturnCode_t Beeper::onInitialize()
{
  std::cerr << "[" << m_profile.instance_name << "] : onInitialize()" << std::endl;
  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  bindParameter("debugLevel", m_debugLevel, "0");
  
  // </rtc-template>

  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("beepCommand", m_beepCommandIn);

  // Set OutPort buffer

  // Set service consumers to Ports

  // Set CORBA Service Ports

  // </rtc-template>

  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t Beeper::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t Beeper::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t Beeper::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

RTC::ReturnCode_t Beeper::onActivated(RTC::UniqueId ec_id)
{
  std::cerr << "[" << m_profile.instance_name << "] : onActivated(" << ec_id << ")" << std::endl;
  return RTC::RTC_OK;
}

RTC::ReturnCode_t Beeper::onDeactivated(RTC::UniqueId ec_id)
{
  std::cerr << "[" << m_profile.instance_name << "] : onDeactivated(" << ec_id << ")" << std::endl;
  return RTC::RTC_OK;
}

#define DEBUGP ((m_debugLevel==1 && m_loop%200==0) || m_debugLevel > 1 )
RTC::ReturnCode_t Beeper::onExecute(RTC::UniqueId ec_id)
{
  // std::cout << m_profile.instance_name<< ": onExecute(" << ec_id << "), data = " << m_data.data << std::endl;
  m_loop++;

  if (m_beepCommandIn.isNew()) {
    m_beepCommandIn.read();
    pthread_mutex_lock( &beep_mutex );
    beep_start = (m_beepCommand.data[BEEP_INFO_START] == 1);
    beep_freq = m_beepCommand.data[BEEP_INFO_FREQ];
    beep_length = m_beepCommand.data[BEEP_INFO_LENGTH];
    pthread_mutex_unlock( &beep_mutex );
  } else {
    pthread_mutex_lock( &beep_mutex );
    beep_start = false;
    pthread_mutex_unlock( &beep_mutex );
  }
  if (DEBUGP) {
    std::cerr << "[" << m_profile.instance_name<< "] start " << beep_start << " freq " << beep_freq << " length " << beep_length << std::endl;
  }
//   if (beep_start) {
//     start_beep(beep_freq, beep_length, true);
//   } else {
//     stop_beep(true);
//   }

  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t Beeper::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t Beeper::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t Beeper::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t Beeper::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t Beeper::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


extern "C"
{

  void BeeperInit(RTC::Manager* manager)
  {
    RTC::Properties profile(beeper_spec);
    manager->registerFactory(profile,
                             RTC::Create<Beeper>,
                             RTC::Delete<Beeper>);
  }

};


