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

// Module specification
// <rtc-template block="module_spec">
static const char* thermoestimator_spec[] =
{
  "implementation_id", "ThermoEstimator",
  "type_name",         "ThermoEstimator",
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

ThermoEstimator::ThermoEstimator(RTC::Manager* manager)
  : RTC::DataFlowComponentBase(manager),
    // <rtc-template block="initializer">
    m_tauInIn("tauIn", m_tauIn),
    m_qRefInIn("qRefIn", m_qRefIn),
    m_qCurrentInIn("qCurrentIn", m_qCurrentIn),
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
  std::cerr << "[" << m_profile.instance_name << "] : onInitialize()" << std::endl;
  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  bindParameter("debugLevel", m_debugLevel, "0");
  
  // </rtc-template>

  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("tauIn", m_tauInIn);
  addInPort("qRefIn", m_qRefInIn);
  addInPort("qCurrentIn", m_qCurrentInIn);
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
    std::cerr << "[" << m_profile.instance_name << "] failed to load model[" << prop["model"] << "]"
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
  std::cerr << "[" << m_profile.instance_name << "] : ambient temperature: " << m_ambientTemp << std::endl;
  
  // set motor heat parameters
  m_motorHeatParams.resize(m_robot->numJoints());
  coil::vstring motorHeatParamsFromConf = coil::split(prop["motor_heat_params"], ",");
  if (motorHeatParamsFromConf.size() != 2 * m_robot->numJoints()) {
    std::cerr << "[" << m_profile.instance_name << "] [WARN]: size of motorHeatParams is " << motorHeatParamsFromConf.size() << ", not equal to 2 * " << m_robot->numJoints() << std::endl;
    // motorHeatParam has default values itself
  } else {
    for (int i = 0; i < m_robot->numJoints(); i++) {
      m_motorHeatParams[i].temperature = m_ambientTemp;
      coil::stringTo(m_motorHeatParams[i].currentCoeffs, motorHeatParamsFromConf[2 * i].c_str());
      coil::stringTo(m_motorHeatParams[i].thermoCoeffs, motorHeatParamsFromConf[2 * i + 1].c_str());
    }
    if (m_debugLevel > 0) {
      std::cerr <<  "motorHeatParams is " << std::endl;
      for (int i = 0; i < m_robot->numJoints(); i++) {
        std::cerr << m_motorHeatParams[i].currentCoeffs << " " << m_motorHeatParams[i].thermoCoeffs << std::endl;
      }
    }
  }

  // set constant for joint error to torque conversion
  coil::vstring error2tauFromConf = coil::split(prop["error_to_tau_constant"], ",");
  if (error2tauFromConf.size() != m_robot->numJoints()) {
    std::cerr << "[" << m_profile.instance_name << "] [WARN]: size of error2tau is " << error2tauFromConf.size() << ", not equal to " << m_robot->numJoints() << std::endl;
    m_error2tau.resize(0); // invalid 
  } else {
    m_error2tau.resize(m_robot->numJoints());
    for (int i = 0; i < m_robot->numJoints(); i++) {
      coil::stringTo(m_error2tau[i], error2tauFromConf[i].c_str());
    }
    if (m_debugLevel > 0) {
      std::cerr <<  "motorHeatParams:" << std::endl;
      for (int i = 0; i < m_robot->numJoints(); i++) {
        std::cerr << " " << m_error2tau[i];
      }
      std::cerr << std::endl;      
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
  std::cerr << "[" << m_profile.instance_name << "] : onActivated(" << ec_id << ")" << std::endl;
  return RTC::RTC_OK;
}

RTC::ReturnCode_t ThermoEstimator::onDeactivated(RTC::UniqueId ec_id)
{
  std::cerr << "[" << m_profile.instance_name << "] : onDeactivated(" << ec_id << ")" << std::endl;
  return RTC::RTC_OK;
}

RTC::ReturnCode_t ThermoEstimator::onExecute(RTC::UniqueId ec_id)
{
  // std::cout << m_profile.instance_name<< ": onExecute(" << ec_id << ")" << std::endl;
  int numJoints = m_robot->numJoints();
  m_loop++;
  
  coil::TimeValue coiltm(coil::gettimeofday());
  RTC::Time tm;
  tm.sec = coiltm.sec();
  tm.nsec = coiltm.usec()*1000;

  if (m_tauInIn.isNew()) {
    m_tauInIn.read();
  }
  if (m_qRefInIn.isNew()) {
    m_qRefInIn.read();
  }
  if (m_qCurrentInIn.isNew()) {
    m_qCurrentInIn.read();
  }
  if (m_servoStateInIn.isNew()) {
    m_servoStateInIn.read();
  }

  // calculate joint torque
  hrp::dvector jointTorque;
  if (m_tauIn.data.length() == numJoints) { // use raw torque
    jointTorque.resize(numJoints);
    for (int i = 0; i < numJoints; i++) {
      jointTorque[i] = m_tauIn.data[i];
    }
    if (isDebug()) {
      std::cerr << "raw torque: ";
      for (int i = 0; i < numJoints; i++) {
        std::cerr << " " << m_tauIn.data[i] ;
      }
      std::cerr << std::endl;
    }
  } else if (m_qRefIn.data.length() == numJoints
             && m_qCurrentIn.data.length() == numJoints) { // estimate torque from joint error
    jointTorque.resize(numJoints);
    hrp::dvector jointError(numJoints);
    for (int i = 0; i < numJoints; i++) {
      jointError[i] = m_qRefIn.data[i] - m_qCurrentIn.data[i];
    }
    estimateJointTorqueFromJointError(jointError, jointTorque);
    if (isDebug()) {
      std::cerr << "qRef: ";
      for (int i = 0; i < numJoints; i++) {
        std::cerr << " " << m_qRefIn.data[i] ;
      }
      std::cerr << std::endl;
      std::cerr << "qCurrent: ";
      for (int i = 0; i < numJoints; i++) {
        std::cerr << " " << m_qCurrentIn.data[i] ;
      }
      std::cerr << std::endl;
    }
  } else { // invalid 
    jointTorque.resize(0);
  }

  // calculate temperature from joint torque
  if (jointTorque.size() ==  m_robot->numJoints()) {
    for (int i = 0; i < numJoints; i++) {
      // Thermo estimation
      calculateJointTemperature(jointTorque[i], m_motorHeatParams[i]);
      // output
      m_tempOut.data[i] = m_motorHeatParams[i].temperature;
    }
    if (isDebug()) {
      std::cerr << std::endl << "temperature  : ";
      for (int i = 0; i < numJoints; i++) {
        std::cerr << " " << m_motorHeatParams[i].temperature;
      }
      std::cerr << std::endl;
    }
    m_tempOutOut.write();
  }

  // overwrite temperature in servoState if temperature is calculated correctly
  if (jointTorque.size() == m_robot->numJoints()
      && m_servoStateIn.data.length() ==  m_robot->numJoints()) {
    for (unsigned int i = 0; i < m_servoStateIn.data.length(); i++) {
      size_t len = m_servoStateIn.data[i].length();
      m_servoStateOut.data[i].length(len + 1); // expand extra_data for temperature
      for (unsigned int j = 0; j < len; j++) {
        m_servoStateOut.data[i][j] = m_servoStateIn.data[i][j];
      }
      // servoStateOut is int, but extra data will be casted to float in HrpsysSeqStateROSBridge
      float tmp_temperature = static_cast<float>(m_motorHeatParams[i].temperature);
      std::memcpy(&(m_servoStateOut.data[i][len]), &tmp_temperature, sizeof(float));
    }
  } else { // pass servoStateIn to servoStateOut
    m_servoStateOut.data.length(m_servoStateIn.data.length());
    for (int i = 0; i < m_servoStateIn.data.length(); i++) {
      m_servoStateOut.data[i] = m_servoStateIn.data[i];
    }
  }
  m_servoStateOutOut.write();
  
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

void ThermoEstimator::estimateJointTorqueFromJointError(hrp::dvector &error, hrp::dvector &tau)
{
  if (error.size() == m_robot->numJoints()
      && m_error2tau.size() == m_robot->numJoints()) {
    tau.resize(m_robot->numJoints());
    for (int i = 0; i < m_robot->numJoints(); i++) {
      tau[i] = m_error2tau[i] * error[i];
    }
    if (isDebug()) {
      std::cerr << "estimated torque: ";
      for (int i = 0; i < m_robot->numJoints(); i++) {
        std::cerr << " " << tau[i] ;
      }
      std::cerr << std::endl;
    }
  } else {
    tau.resize(0); // don't calculate tau when invalid input
    if (isDebug()) {
      std::cerr << "Invalid size of values:" << std::endl;
      std::cerr << "num joints: " << m_robot->numJoints() << std::endl;
      std::cerr << "joint error: " << error.size() << std::endl;
      std::cerr << "error2tau: " << m_error2tau.size() << std::endl;
    }
  }
  
  return;
}

void ThermoEstimator::calculateJointTemperature(double tau, MotorHeatParam& param)
{
  // from Design of High Torque and High Speed Leg Module for High Power Humanoid (Junichi Urata et al.)
  // Tnew = T + (((Re*K^2/C) * tau^2) - ((1/RC) * (T - Ta))) * dt
  double currentHeat, radiation;
  currentHeat = param.currentCoeffs * std::pow(tau, 2);
  radiation = -param.thermoCoeffs * (param.temperature - m_ambientTemp);
  param.temperature = param.temperature + (currentHeat + radiation) * m_dt;
  return;
}

bool ThermoEstimator::isDebug(int cycle)
{
  return ((m_debugLevel==1 && m_loop%cycle==0) || m_debugLevel > 1);
}

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


