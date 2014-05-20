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

#define DEBUGP ((m_debugLevel==1 && loop%200==0) || m_debugLevel > 1 )
#define DQ_MAX 1.0

// Module specification
// <rtc-template block="module_spec">
static const char* thermolimiter_spec[] =
  {
    "implementation_id", "ThermoLimiter",
    "type_name",         "ThermoLimiter",
    "description",       "null component",
    "version",           HRPSYS_PACKAGE_VERSION,
    "vendor",            "AIST",
    "category",          "example",
    "activity_type",     "DataFlowComponent",
    "max_instance",      "10",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    "conf.default.string", "test",
    "conf.default.intvec", "1,2,3",
    "conf.default.double", "1.234",

    ""
  };
// </rtc-template>

ThermoLimiter::ThermoLimiter(RTC::Manager* manager)
  : RTC::DataFlowComponentBase(manager),
    // <rtc-template block="initializer">
    m_tempInIn("tempIn", m_tempIn),
    m_tauInIn("tauIn", m_tauIn),
    m_qCurrentInIn("qCurrentIn", m_qCurrentIn),
    m_tauMaxOutOut("tauMax", m_tauMaxOut),
    m_debugLevel(0)
{
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
  addInPort("tauIn", m_tauInIn);
  addInPort("qCurrentIn", m_qCurrentInIn);

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
  if (motorTemperatureLimitFromConf.size() != m_robot->numJoints()) {
    std::cerr <<  "[WARN]: size of motor_temperature_limit is " << motorTemperatureLimitFromConf.size() << ", not equal to " << m_robot->numJoints() << std::endl;
    m_motorTemperatureLimit.resize(m_robot->numJoints(), 100.0);
  } else {
    m_motorTemperatureLimit.resize(m_robot->numJoints());
    for (int i = 0; i < m_robot->numJoints(); i++) {
      coil::stringTo(m_motorTemperatureLimit[i], motorTemperatureLimitFromConf[i].c_str());
    }
  }
  if (m_debugLevel > 0) {
    std::cerr <<  "motor_temperature_limit: ";
    for(std::vector<double>::iterator it = m_motorTemperatureLimit.begin(); it != m_motorTemperatureLimit.end(); ++it){
      std::cerr << *it << " ";
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
  static long long loop = 0;
  loop ++;

  coil::TimeValue coiltm(coil::gettimeofday());
  RTC::Time tm;
  tm.sec = coiltm.sec();
  tm.nsec = coiltm.usec()*1000;
  bool isTempError = false;
  hrp::dvector tauMax;
  tauMax.resize(m_robot->numJoints());

  // update port
  if (m_tempInIn.isNew()) {
    m_tempInIn.read();
  }
  if (m_tauInIn.isNew()) {
    m_tauInIn.read();
  }
  if (m_qCurrentInIn.isNew()) {
    m_qCurrentInIn.read();
  }

  // calculate tauMax
  isTempError = limitTemperature(tauMax);
  // call beep
  if (isTempError) {
    start_beep(3136);
  } else {
    stop_beep();
  }
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

bool ThermoLimiter::limitTemperature(hrp::dvector &tauMax)
{
  static long loop = 0;
  loop++;
  
  int numJoints = m_robot->numJoints();
  bool isTempError = false;
  double temp, tempLimit;
  hrp::dvector squareTauMax(numJoints);

  if (DEBUGP) {
    std::cerr << "[" << m_profile.instance_name << "]" << std::endl;
  }
  
  if ( m_tempIn.data.length() ==  m_robot->numJoints() ) {
    if (DEBUGP) {
      std::cerr << "temperature: ";
      for (int i = 0; i < numJoints; i++) {
        std::cerr << " " << m_tempIn.data[i];
      }
      std::cerr << std::endl;
      std::cerr << "tauIn: ";
      for (int i = 0; i < numJoints; i++) {
        std::cerr << " " << m_tauIn.data[i];
      }
      std::cerr << std::endl;
    }

    for (int i = 0; i < numJoints; i++) {
      temp = m_tempIn.data[i];
      tempLimit = m_motorTemperatureLimit[i];

      // limit temperature
      double term = 120;
      squareTauMax[i] = (((tempLimit - temp) / term) + m_motorHeatParams[i].thermoCoeffs * (temp - m_motorHeatParams[i].temperature)) / m_motorHeatParams[i].currentCoeffs;

      // determine tauMax
      if (squareTauMax[i] < 0) {
        std::cerr << "[WARN] tauMax ** 2 = " << squareTauMax[i] << " < 0 in Joint " << i << std::endl;
        tauMax[i] = m_robot->joint(i)->climit;
      } else {
        if (std::pow(m_tauIn.data[i], 2) > squareTauMax[i]) {
          std::cerr << "[WARN] tauMax over in Joint " << i << ": " << m_tauIn.data[i] << "^2 > " << squareTauMax[i] << std::endl;
          isTempError = true; // in thermo emergency
        }
        tauMax[i] = std::sqrt(squareTauMax[i]);
      }
    }
    if (DEBUGP) {
      std::cerr << std::endl;
      std::cerr << "tauMax: ";
      for (int i = 0; i < tauMax.size(); i++) {
        std::cerr << tauMax[i] << " ";
      }
      std::cerr << std::endl;
    }
  }
  return isTempError;
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


