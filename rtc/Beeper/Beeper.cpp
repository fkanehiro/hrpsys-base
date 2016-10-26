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
#include<deque>

// Variables for beep_thread
struct BeepData
{
  bool _beep_start;
  int _beep_freq;
  int _beep_length;
};

bool is_initialized = false;
pthread_mutex_t beep_mutex;  // Mutex
double m_dt = 0.002;
std::deque<BeepData> beep_command_buffer; // Used for communication between beepthread and real-time thread

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
  // Initialize
  init_beep();
  bool wait_for_initialized = true;
  while (wait_for_initialized) { // Wait until m_dt is set
    usleep(2000);
    pthread_mutex_lock( &beep_mutex );
    wait_for_initialized = !is_initialized;
    pthread_mutex_unlock( &beep_mutex );
  }
  // Loop
  bool prev_beep_start=false;
  while (1) {
    usleep(static_cast<size_t>(1000000*m_dt));
    // Get beepCommand from buffer
    pthread_mutex_lock( &beep_mutex );
    BeepData bd = beep_command_buffer.front();
    if (beep_command_buffer.size() > 1) beep_command_buffer.pop_front();
    pthread_mutex_unlock( &beep_mutex );
//     if (!prev_beep_start && tmp_beep_start) {
//       std::cerr << "BP START" << std::endl;
//     } else if (prev_beep_start && !tmp_beep_start) {
//       std::cerr << "BP STOP" << std::endl;
//     }
    // Beep
    if (bd._beep_start) {
      start_beep(bd._beep_freq, bd._beep_length);
    } else if (prev_beep_start) { // If !beep_start and prev_beep_start, stop_beep just once.
      stop_beep();
    }
    prev_beep_start = bd._beep_start;
  }
  quit_beep();
}

Beeper::Beeper(RTC::Manager* manager)
  : RTC::DataFlowComponentBase(manager),
    // <rtc-template block="initializer">
    m_beepCommandIn("beepCommand", m_beepCommand),
    m_loop(0),
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

  RTC::Properties& prop =  getProperties();
  coil::stringTo(m_dt, prop["dt"].c_str());
  pthread_mutex_lock( &beep_mutex );
  is_initialized = true;
  pthread_mutex_unlock( &beep_mutex );
  std::cerr << "[" << m_profile.instance_name << "] : Beep thread dt = " << m_dt << "[s]" << std::endl;

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
    // Read beepCommand from data port
    m_beepCommandIn.read();
    BeepData bd;
    bd._beep_start = (m_beepCommand.data[BEEP_INFO_START] == 1);
    bd._beep_freq = m_beepCommand.data[BEEP_INFO_FREQ];
    bd._beep_length = m_beepCommand.data[BEEP_INFO_LENGTH];
    // Push beepCommand to buffer
    size_t max_buffer_length = 10;
    if (pthread_mutex_trylock( &beep_mutex ) == 0) {
        beep_command_buffer.push_back(bd);
        while (beep_command_buffer.size() > max_buffer_length) beep_command_buffer.pop_front();
        pthread_mutex_unlock( &beep_mutex );
    } else {
        std::cerr << "[" << m_profile.instance_name<< "] Mutex trylock failed (loop=" << m_loop << ")" << std::endl;
    }
    // print
    if (m_debugLevel > 0) {
      if (bd._beep_start)
        std::cerr << "[" << m_profile.instance_name<< "] isNew : beep start (freq=" << bd._beep_freq << ", length=" << bd._beep_length << ", loop=" << m_loop << ")" << std::endl;
      else
        std::cerr << "[" << m_profile.instance_name<< "] isNew : beep stop (loop=" << m_loop << ")" << std::endl;
    }
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


