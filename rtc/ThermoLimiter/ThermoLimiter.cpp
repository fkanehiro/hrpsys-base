// -*- C++ -*-
/*!
 * @file  ThermoLimiter.cpp
 * @brief null component
 * $Date$
 *
 * $Id$
 */

#include "ThermoLimiter.h"
#include "../SoftErrorLimiter/beep.h"
#include <rtm/CorbaNaming.h>
#include <hrpModel/ModelLoaderUtil.h>
#include <hrpUtil/MatrixSolvers.h>
#include <cmath>

#define DQ_MAX 1.0

// Module specification
// <rtc-template block="module_spec">
static const char* thermolimiter_spec[] =
  {
    "implementation_id", "ThermoLimiter",
    "type_name",         "ThermoLimiter",
    "description",       "thermo limiter",
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

ThermoLimiter::ThermoLimiter(RTC::Manager* manager)
  : RTC::DataFlowComponentBase(manager),
    // <rtc-template block="initializer">
    m_tempInIn("tempIn", m_tempIn),
    m_tauMaxOutOut("tauMax", m_tauMaxOut),
    m_debugLevel(0)
{
  init_beep();
}

ThermoLimiter::~ThermoLimiter()
{
}

RTC::ReturnCode_t ThermoLimiter::onInitialize()
{
  std::cout << m_profile.instance_name << ": onInitialize()" << std::endl;
  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  bindParameter("debugLevel", m_debugLevel, "0");
  
  // </rtc-template>

  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("tempIn", m_tempInIn);

  // Set OutPort buffer
  addOutPort("tauMax", m_tauMaxOutOut);
  
  // Set service provider to Ports
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  
  // </rtc-template>

  RTC::Properties& prop = getProperties();
  coil::stringTo(m_dt, prop["dt"].c_str());

  m_robot = hrp::BodyPtr(new hrp::Body());

  RTC::Manager& rtcManager = RTC::Manager::instance();
  std::string nameServer = rtcManager.getConfig()["corba.nameservers"];
  int comPos = nameServer.find(",");
  if (comPos < 0){
    comPos = nameServer.length();
  }
  nameServer = nameServer.substr(0, comPos);
  RTC::CorbaNaming naming(rtcManager.getORB(), nameServer.c_str());
  if (!loadBodyFromModelLoader(m_robot, prop["model"].c_str(),
                               CosNaming::NamingContext::_duplicate(naming.getRootContext())
        )){
    std::cerr << "failed to load model[" << prop["model"] << "]"
              << std::endl;
  }
  // set limit of motor temperature
  coil::vstring motorTemperatureLimitFromConf = coil::split(prop["motor_temperature_limit"], ",");
  m_motorTemperatureLimit.resize(m_robot->numJoints());
  if (motorTemperatureLimitFromConf.size() != m_robot->numJoints()) {
    std::cerr <<  "[WARN]: size of motor_temperature_limit is " << motorTemperatureLimitFromConf.size() << ", not equal to " << m_robot->numJoints() << std::endl;
    for (int i = 0; i < m_robot->numJoints(); i++) {
      m_motorTemperatureLimit[i] = 80.0;
    }
  } else {
    for (int i = 0; i < m_robot->numJoints(); i++) {
      coil::stringTo(m_motorTemperatureLimit[i], motorTemperatureLimitFromConf[i].c_str());
    }
  }
  if (m_debugLevel > 0) {
    std::cerr <<  "motor_temperature_limit: ";
    for(int i = 0; i < m_motorTemperatureLimit.size(); i++) {
      std::cerr << m_motorTemperatureLimit[i] << " ";
    }
    std::cerr << std::endl;
  }

  // set temperature of environment
  double ambientTemp = 25.0;
  if (prop["ambient_tmp"] != "") {
    coil::stringTo(ambientTemp, prop["ambient_tmp"].c_str());
  }
  std::cerr <<  m_profile.instance_name << ": ambient temperature: " << ambientTemp << std::endl;

  // set limit of motor heat parameters
  coil::vstring motorHeatParamsFromConf = coil::split(prop["motor_heat_params"], ",");
  m_motorHeatParams.resize(m_robot->numJoints());
  if (motorHeatParamsFromConf.size() != 2 * m_robot->numJoints()) {
    std::cerr <<  "[WARN]: size of motor_heat_param is " << motorHeatParamsFromConf.size() << ", not equal to 2 * " << m_robot->numJoints() << std::endl;
    for (int i = 0; i < m_robot->numJoints(); i++) {
      m_motorHeatParams[i].defaultParams();
      m_motorHeatParams[i].temperature = ambientTemp;
    }

  } else {
    for (int i = 0; i < m_robot->numJoints(); i++) {
      m_motorHeatParams[i].temperature = ambientTemp;
      coil::stringTo(m_motorHeatParams[i].currentCoeffs, motorHeatParamsFromConf[2 * i].c_str());
      coil::stringTo(m_motorHeatParams[i].thermoCoeffs, motorHeatParamsFromConf[2 * i + 1].c_str());
    }
  }
  
  if (m_debugLevel > 0) {
    std::cerr <<  "motor_heat_param: ";
    for(std::vector<MotorHeatParam>::iterator it = m_motorHeatParams.begin(); it != m_motorHeatParams.end(); ++it){
      std::cerr << (*it).temperature << "," << (*it).currentCoeffs << "," << (*it).thermoCoeffs << ", ";
    }
    std::cerr << std::endl;
    std::cerr << "default climit value:" << std::endl;
    for (int i = 0; i < m_robot->numJoints(); i++) {
      std::cerr << m_robot->joint(i)->name << ":" << m_robot->joint(i)->climit << std::endl;
    }
  }

  // allocate memory for outPorts
  m_tauMaxOut.data.length(m_robot->numJoints());
  
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t ThermoLimiter::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t ThermoLimiter::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t ThermoLimiter::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

RTC::ReturnCode_t ThermoLimiter::onActivated(RTC::UniqueId ec_id)
{
  std::cout << m_profile.instance_name<< ": onActivated(" << ec_id << ")" << std::endl;
  return RTC::RTC_OK;
}

RTC::ReturnCode_t ThermoLimiter::onDeactivated(RTC::UniqueId ec_id)
{
  std::cout << m_profile.instance_name<< ": onDeactivated(" << ec_id << ")" << std::endl;
  return RTC::RTC_OK;
}

RTC::ReturnCode_t ThermoLimiter::onExecute(RTC::UniqueId ec_id)
{
  // std::cout << m_profile.instance_name<< ": onExecute(" << ec_id << "), data = " << m_data.data << std::endl;
  m_loop++;

  if (isDebug()) {
    std::cerr << "[" << m_profile.instance_name << "]" << std::endl;
  }
  
  coil::TimeValue coiltm(coil::gettimeofday());
  RTC::Time tm;
  tm.sec = coiltm.sec();
  tm.nsec = coiltm.usec()*1000;
  hrp::dvector tauMax;
  tauMax.resize(m_robot->numJoints());

  double alarmRatio = 0.5;
  double thermoLimitRatio = 0.0;
  double torqueLimitRatio = 0.0;
  std::string thermoLimitPrefix = "ThermoLimit";
  
  // update port
  if (m_tempInIn.isNew()) {
    m_tempInIn.read();
  }

  if (isDebug()) {
    std::cerr << "temperature: ";
    for (int i = 0; i < m_tempIn.data.length(); i++) {
      std::cerr << " " << m_tempIn.data[i];
    }
    std::cerr << std::endl;
  }
 
  // calculate tauMax from temperature
  if (m_tempIn.data.length() == m_robot->numJoints()) {
    calcMaxTorqueFromTemperature(tauMax);
  } else {
    for (int i = 0; i < m_robot->numJoints(); i++) {
      tauMax[i] = m_robot->joint(i)->climit; // default torque limit from model
    }
  }

  if (isDebug()) {
    std::cerr << "tauMax: ";
    for (int i = 0; i < tauMax.size(); i++) {
      std::cerr << tauMax[i] << " ";
    }
    std::cerr << std::endl;
  }

  // emergency notification
  if (m_tempIn.data.length() == m_robot->numJoints()) {
    thermoLimitRatio = calcEmergencyRatio(m_tempIn, m_motorTemperatureLimit, alarmRatio, thermoLimitPrefix);
  }

  // call beep (3136/0.8=3920)
  callBeep(thermoLimitRatio, alarmRatio);
  
  // output restricted tauMax
  for (int i = 0; i < m_robot->numJoints(); i++) {
    m_tauMaxOut.data[i] = tauMax[i];
  }
  m_tauMaxOut.tm = tm;
  m_tauMaxOutOut.write();
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t ThermoLimiter::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t ThermoLimiter::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t ThermoLimiter::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t ThermoLimiter::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t ThermoLimiter::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

void ThermoLimiter::calcMaxTorqueFromTemperature(hrp::dvector &tauMax)
{
  int numJoints = m_robot->numJoints();
  double temp, tempLimit;
  hrp::dvector squareTauMax(numJoints);
  
  if (m_tempIn.data.length() ==  m_robot->numJoints()) {

    for (int i = 0; i < numJoints; i++) {
      temp = m_tempIn.data[i];
      tempLimit = m_motorTemperatureLimit[i];

      // limit temperature
      double term = 120;
      squareTauMax[i] = (((tempLimit - temp) / term) + m_motorHeatParams[i].thermoCoeffs * (temp - m_motorHeatParams[i].temperature)) / m_motorHeatParams[i].currentCoeffs;

      // determine tauMax
      if (squareTauMax[i] < 0) {
        std::cerr << "[WARN] tauMax ** 2 = " << squareTauMax[i] << " < 0 in Joint " << i << std::endl;
        tauMax[i] = m_robot->joint(i)->climit; // default tauMax from model file
      } else {
        tauMax[i] = std::sqrt(squareTauMax[i]); // tauMax is absolute value
      }
    }
  }
  return;
}

double ThermoLimiter::calcEmergencyRatio(RTC::TimedDoubleSeq &current, hrp::dvector &max, double alarmRatio, std::string &prefix)
{
  double maxEmergencyRatio = 0.0;
  if (current.data.length() == max.size()) { // estimate same dimension
    for (int i = 0; i < current.data.length(); i++) {
      double tmpEmergencyRatio = std::abs(current.data[i] / max[i]);
      if (tmpEmergencyRatio > alarmRatio) {
        std::cerr << prefix << "[" << m_robot->joint(i)->name << "]" << " is over " << alarmRatio << " of the limit.";
        if (m_debugLevel > 0) {
          std::cerr << ": " << current.data[i] << ">" << max[i];
        }
        std::cerr << std::endl;
      }
      if (maxEmergencyRatio < tmpEmergencyRatio) {
        maxEmergencyRatio = tmpEmergencyRatio;
      }
    }
  }
  return maxEmergencyRatio;
}

void ThermoLimiter::callBeep(double ratio, double alarmRatio)
{
  const int maxFreq = 3136; // G
  const int minFreq = 2794; // F

  if (ratio > 1.0) { // emergency (current load is over max load)
    const int emergency_beep_cycle = 200;
    int current_emergency_beep_cycle = m_loop % emergency_beep_cycle;
    if (current_emergency_beep_cycle <= (emergency_beep_cycle / 2)) {
      start_beep(4000, 60);
    } else {
      start_beep(2000, 60);
    }
  } else if (ratio > alarmRatio) { // normal warning
    int freq = minFreq + (maxFreq - minFreq) * ((ratio - alarmRatio) / (1.0 - alarmRatio));
    start_beep(freq, 500);
  } else {
    stop_beep();
  }
  return;
}

bool ThermoLimiter::isDebug(int cycle)
{
  return ((m_debugLevel==1 && m_loop%cycle==0) || m_debugLevel > 1);
}

extern "C"
{

  void ThermoLimiterInit(RTC::Manager* manager)
  {
    RTC::Properties profile(thermolimiter_spec);
    manager->registerFactory(profile,
                             RTC::Create<ThermoLimiter>,
                             RTC::Delete<ThermoLimiter>);
  }

};


