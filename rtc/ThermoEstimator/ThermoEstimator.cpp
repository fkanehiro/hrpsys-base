// -*- C++ -*-
/*!
 * @file  ThermoEstimator.cpp
 * @brief motor thermo estimation  component
 *
 *  Design of High Torque and High Speed Leg Module for High Power Humanoid
 *  Junichi Urata et al., IROS 2010, pp 4497 - 4502
 *  Tnew = T + (((Re*K^2/C) * tau^2) - ((1/RC) * (T - Ta))) * dt
 *
 * $Date$
 *
 * $Id$
 */

#include "ThermoEstimator.h"
#include "RobotHardwareService.hh"
#include <rtm/CorbaNaming.h>
#include <hrpModel/ModelLoaderUtil.h>
#include <hrpUtil/MatrixSolvers.h>

#define DEBUGP ((m_debugLevel==1 && loop%200==0) || m_debugLevel > 1 )

// Module specification
// <rtc-template block="module_spec">
static const char* thermoestimator_spec[] =
{
  "implementation_id", "ThermoEstimator",
  "type_name",         "ThermoEstimator",
  "description",       "null component",
  "version",           "1.0",
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

ThermoEstimator::ThermoEstimator(RTC::Manager* manager)
  : RTC::DataFlowComponentBase(manager),
    // <rtc-template block="initializer">
    m_tauInIn("tauIn", m_tauIn),
    m_servoStateInIn("servoStateIn", m_servoStateIn),
    m_tempOutOut("tempOut", m_tempOut),
    m_servoStateOutOut("servoStateOut", m_servoStateOut),    
    // </rtc-template>
    m_debugLevel(0)
{
}

ThermoEstimator::~ThermoEstimator()
{
}

RTC::ReturnCode_t ThermoEstimator::onInitialize()
{
  std::cout << m_profile.instance_name << ": onInitialize()" << std::endl;
  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  bindParameter("debugLevel", m_debugLevel, "0");
  
  // </rtc-template>

  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("tauIn", m_tauInIn);
  addInPort("servoStateIn", m_servoStateInIn);

  // Set OutPort buffer
  addOutPort("tempOut", m_tempOutOut);
  addOutPort("servoStateOut", m_servoStateOutOut);
  
  // Set service provider to Ports
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  
  // </rtc-template>

  // set parmeters
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

  // init outport
  m_tempOut.data.length(m_robot->numJoints());
  m_servoStateIn.data.length(m_robot->numJoints());
  m_servoStateOut.data.length(m_robot->numJoints());
  
  // set temperature of environment
  if (prop["ambient_tmp"] != "") {
    coil::stringTo(m_ambientTemp, prop["ambient_tmp"].c_str());
  } else {
    m_ambientTemp = 25.0;
  }
  std::cerr <<  m_profile.instance_name << ": ambient temperature: " << m_ambientTemp << std::endl;
  
  // set motor heat parameters
  m_motorHeatParams.resize(m_robot->numJoints());
  coil::vstring motorHeatParamsFromConf = coil::split(prop["motor_heat_params"], ",");
  if (motorHeatParamsFromConf.size() != 2 * m_robot->numJoints()) {
    std::cerr <<  "[WARN]: size of motorHeatParams is " << motorHeatParamsFromConf.size() << ", not equal to 2 * " << m_robot->numJoints() << std::endl;
  } else {
    std::cerr <<  "motorHeatParams is " << std::endl;
    for (int i = 0; i < m_robot->numJoints(); i++) {
      m_motorHeatParams[i].temperature = m_ambientTemp;
      std::cerr << motorHeatParamsFromConf[2 * i].c_str() << " " << motorHeatParamsFromConf[2 * i + 1].c_str() << std::endl;
      coil::stringTo(m_motorHeatParams[i].currentCoeffs, motorHeatParamsFromConf[2 * i].c_str());
      coil::stringTo(m_motorHeatParams[i].thermoCoeffs, motorHeatParamsFromConf[2 * i + 1].c_str());
    }
  }
  
  return RTC::RTC_OK;
}



/*
  RTC::ReturnCode_t ThermoEstimator::onFinalize()
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t ThermoEstimator::onStartup(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t ThermoEstimator::onShutdown(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

RTC::ReturnCode_t ThermoEstimator::onActivated(RTC::UniqueId ec_id)
{
  std::cout << m_profile.instance_name<< ": onActivated(" << ec_id << ")" << std::endl;
  return RTC::RTC_OK;
}

RTC::ReturnCode_t ThermoEstimator::onDeactivated(RTC::UniqueId ec_id)
{
  std::cout << m_profile.instance_name<< ": onDeactivated(" << ec_id << ")" << std::endl;
  return RTC::RTC_OK;
}

RTC::ReturnCode_t ThermoEstimator::onExecute(RTC::UniqueId ec_id)
{
  // std::cout << m_profile.instance_name<< ": onExecute(" << ec_id << ")" << std::endl;
  static long long loop = 0;
  loop ++;

  coil::TimeValue coiltm(coil::gettimeofday());
  RTC::Time tm;
  tm.sec = coiltm.sec();
  tm.nsec = coiltm.usec()*1000;

  if (m_tauInIn.isNew()) {
    m_tauInIn.read();
  }
  
  if ( m_tauIn.data.length() ==  m_robot->numJoints() ) {
    int numJoints = m_robot->numJoints();
    if ( DEBUGP ) {
      std::cerr << "raw torque: ";
      for (int i = 0; i < numJoints; i++) {
        std::cerr << " " << m_tauIn.data[i] ;
      }
      std::cerr << std::endl;
    }
    if ( DEBUGP ) {
      std::cerr << "estimation values: " << std::endl;
    }
    for (int i = 0; i < numJoints; i++) {
      MotorHeatParam param = m_motorHeatParams[i];
      double tau, currentHeat, radiation;
      // Thermo estimation
      // from Design of High Torque and High Speed Leg Module for High Power Humanoid (Junichi Urata et al.)
      // Tnew = T + (((Re*K^2/C) * tau^2) - ((1/RC) * (T - Ta))) * dt
      tau = m_tauIn.data[i];
      currentHeat = param.currentCoeffs * std::pow(tau, 2);
      radiation = -param.thermoCoeffs * (param.temperature - m_ambientTemp);
      m_motorHeatParams[i].temperature = param.temperature + (currentHeat + radiation) * m_dt;
      if ( DEBUGP ) {
        std::cerr << currentHeat << " "  << radiation << ", ";
      }
      // output
      m_tempOut.data[i] = m_motorHeatParams[i].temperature;
    }
    if ( DEBUGP ) {
      std::cerr << std::endl << "temperature  : ";
      for (int i = 0; i < numJoints; i++) {
        std::cerr << " " << m_motorHeatParams[i].temperature;
      }
      std::cerr << std::endl;
    }

    // overwrite temperature in servoState
    if ( m_servoStateInIn.isNew() && (m_servoStateIn.data.length() ==  m_robot->numJoints()) ) {
      m_servoStateInIn.read();
      for (unsigned int i = 0; i < m_servoStateIn.data.length(); i++){
        size_t len = m_servoStateIn.data[i].length();
        m_servoStateOut.data[i].length(len + 1); // expand extra_data for temperature
        for (unsigned int j = 0; j < len; j++){
          m_servoStateOut.data[i][j] = m_servoStateIn.data[i][j];
        }
        // servoStateOut is int, but extra data will be casted to float in HrpsysSeqStateROSBridge
        float tmp_temperature = static_cast<float>(m_motorHeatParams[i].temperature);
        std::memcpy(&(m_servoStateOut.data[i][len]), &tmp_temperature, sizeof(float));
      }
      m_servoStateOutOut.write();
    }
    m_tempOutOut.write();
  }
  return RTC::RTC_OK;
}

/*
  RTC::ReturnCode_t ThermoEstimator::onAborting(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t ThermoEstimator::onError(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t ThermoEstimator::onReset(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t ThermoEstimator::onStateUpdate(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t ThermoEstimator::onRateChanged(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/



extern "C"
{

  void ThermoEstimatorInit(RTC::Manager* manager)
  {
    RTC::Properties profile(thermoestimator_spec);
    manager->registerFactory(profile,
                             RTC::Create<ThermoEstimator>,
                             RTC::Delete<ThermoEstimator>);
  }

};


