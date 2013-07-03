// -*- C++ -*-
/*!
 * @file  ThermoEstimator.cpp
 * @brief null component
 * $Date$
 *
 * $Id$
 */

#include "ThermoEstimator.h"
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
    m_tempOutOut("tempOut", m_tempOut),
    // </rtc-template>
    m_debugLevel(1)
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
  // bindParameter("debugLevel", m_debugLevel, "0");
  
  // </rtc-template>

  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("tauIn", m_tauInIn);

  // Set OutPort buffer
  addOutPort("tempOut", m_tempOutOut);
  
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

  // set
  if (prop["ambient_tmp"] != "") {
    coil::stringTo(m_ambientTemp, prop["ambient_tmp"].c_str());
  } else {
    m_ambientTemp = 25.0;
  }
  std::cerr <<  m_profile.instance_name << ": ambient tempreture: " << m_ambientTemp << std::endl;
  
  // set motor heat parameters
  m_motorHeatParams.resize(m_robot->numJoints());
  coil::vstring motorHeatParamsFromConf = coil::split(prop["motor_heat_params"], ",");
  if (motorHeatParamsFromConf.size() != 2 * m_robot->numJoints()) {
    std::cerr <<  "[WARN]: size of motorHeatParams is " << motorHeatParamsFromConf.size() << ", not equal to 2 * " << m_robot->numJoints() << std::endl;
  } else {
    std::cerr <<  "motorHeatParams is " << std::endl;
    for (int i = 0; i < m_robot->numJoints(); i++) {
      m_motorHeatParams[i].tempreture = m_ambientTemp;
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
    for (int i = 0; i < numJoints; i++) {
      MotorHeatParam param = m_motorHeatParams[i];
      double tau, currentHeat, radiation;
      // thermo estimation
      // Tnew = T + (((Re*K^2/C) * tau^2) - ((1/RC) * (T - Ta))) * dt
      // if (std::abs(m_tauIn.data[i]) > 15.0) {
      //   tau = m_tauIn.data[i];
      // } else {
      //   tau = 0;
      // }
      tau = m_tauIn.data[i];
      currentHeat = param.currentCoeffs * std::pow(tau, 2);
      radiation = -param.thermoCoeffs * (param.tempreture - m_ambientTemp);
      m_motorHeatParams[i].tempreture = param.tempreture + (currentHeat + radiation) * m_dt;
      if (DEBUGP) {
        std::cerr << m_motorHeatParams[i].tempreture << " = " << param.tempreture << " " << currentHeat << " " << radiation << std::endl;
      }
      // output
      m_tempOut.data[i] = m_motorHeatParams[i].tempreture;
    }
    if ( DEBUGP ) {
      std::cerr << "  tempreture  : ";
      for (int i = 0; i < numJoints; i++) {
        std::cerr << " " << m_motorHeatParams[i].tempreture;
      }
      std::cerr << std::endl;
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


