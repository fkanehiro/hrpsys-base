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

static double vlimit(double value, double llimit, double ulimit);

// Module specification
// <rtc-template block="module_spec">
static const char* thermolimiter_spec[] =
  {
    "implementation_id", "ThermoLimiter",
    "type_name",         "ThermoLimiter",
    "description",       "null component",
    "version",           "1.0",
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
    m_qRefInIn("qRefIn", m_qRefIn),
    m_qCurrentInIn("qCurrentIn", m_qCurrentIn),
    m_qRefOutOut("qRefOut", m_qRefOut),
    m_debugLevel(1)
    // </rtc-template>
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
  // bindParameter("string", confstring, "testtest");
  // bindParameter("intvec", confintvec, "4,5,6,7");
  // bindParameter("double", confdouble, "4.567");
  
  // </rtc-template>

  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("tempIn", m_tempInIn);
  addInPort("tauIn", m_tauInIn);
  addInPort("qCurrentIn", m_qCurrentInIn);
  addInPort("qRefIn", m_qRefInIn);

  // Set OutPort buffer
  addOutPort("qRefOut", m_qRefOutOut);
  
  // Set service provider to Ports
  // m_NullServicePort.registerProvider("service0", "NullService", m_NullService);
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  // addPort(m_NullServicePort);
  
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
  // set limit of motor tempreture
  coil::vstring motorTempretureLimitFromConf = coil::split(prop["motor_tempreture_limit"], ",");
  if (motorTempretureLimitFromConf.size() != m_robot->numJoints()) {
    std::cerr <<  "[WARN]: size of motor_tempreture_limit is " << motorTempretureLimitFromConf.size() << ", not equal to " << m_robot->numJoints() << std::endl;
    m_motorTempretureLimit.resize(m_robot->numJoints(), 100.0);
  } else {
    m_motorTempretureLimit.resize(m_robot->numJoints());
    for (int i = 0; i < m_robot->numJoints(); i++) {
      coil::stringTo(m_motorTempretureLimit[i], motorTempretureLimitFromConf[i].c_str());
    }
  }
  if (m_debugLevel > 0) {
    std::cerr <<  "motor_tempreture_limit: ";
    for(std::vector<double>::iterator it = m_motorTempretureLimit.begin(); it != m_motorTempretureLimit.end(); ++it){
      std::cerr << *it << " ";
    }
    std::cerr << std::endl;
  }

  // set tempreture of environment
  double ambientTemp = 25.0;
  if (prop["ambient_tmp"] != "") {
    coil::stringTo(ambientTemp, prop["ambient_tmp"].c_str());
  }
  std::cerr <<  m_profile.instance_name << ": ambient tempreture: " << ambientTemp << std::endl;

  // set limit of motor heat parameters
  coil::vstring motorHeatParamsFromConf = coil::split(prop["motor_heat_params"], ",");
  m_motorHeatParams.resize(m_robot->numJoints());
  if (motorHeatParamsFromConf.size() != 2 * m_robot->numJoints()) {
    std::cerr <<  "[WARN]: size of motor_heat_param is " << motorHeatParamsFromConf.size() << ", not equal to 2 * " << m_robot->numJoints() << std::endl;
    for (int i = 0; i < m_robot->numJoints(); i++) {
      m_motorHeatParams[i].defaultParams();
      m_motorHeatParams[i].tempreture = ambientTemp;
    }

  } else {
    for (int i = 0; i < m_robot->numJoints(); i++) {
      m_motorHeatParams[i].tempreture = ambientTemp;
      coil::stringTo(m_motorHeatParams[i].currentCoeffs, motorHeatParamsFromConf[2 * i].c_str());
      coil::stringTo(m_motorHeatParams[i].thermoCoeffs, motorHeatParamsFromConf[2 * i + 1].c_str());
    }
  }
  
  if (m_debugLevel > 0) {
    std::cerr <<  "motor_heat_param: ";
    for(std::vector<MotorHeatParam>::iterator it = m_motorHeatParams.begin(); it != m_motorHeatParams.end(); ++it){
      std::cerr << (*it).tempreture << "," << (*it).currentCoeffs << "," << (*it).thermoCoeffs << ", ";
    }
    std::cerr << std::endl;
  }
  
  // make torque controller
  // set limit of motor heat parameters
  coil::vstring torqueControllerParamsFromConf = coil::split(prop["torque_controller_params"], ",");
  m_motorTwoDofControllers.resize(m_robot->numJoints());
  if (torqueControllerParamsFromConf.size() != 2 * m_robot->numJoints()) {
    std::cerr <<  "[WARN]: size of torque_controller_params is " << torqueControllerParamsFromConf.size() << ", not equal to 2 * " << m_robot->numJoints() << std::endl;
    for (std::vector<TwoDofController>::iterator it = m_motorTwoDofControllers.begin(); it != m_motorTwoDofControllers.end() ; ++it) {
      (*it).setup(400.0, 0.04, m_dt); // set default params
      // (*it).setup(400.0, 1.0, m_dt);
    }
  } else {
    double tdcParamK, tdcParamT;
    for (int i = 0; i < m_robot->numJoints(); i++) {
      coil::stringTo(tdcParamK, torqueControllerParamsFromConf[2 * i].c_str());
      coil::stringTo(tdcParamT, torqueControllerParamsFromConf[2 * i + 1].c_str());
      m_motorTwoDofControllers[i].setup(tdcParamK, tdcParamT, m_dt);
    }
  }
  // for (std::vector<TwoDofController>::iterator it = m_motorTwoDofControllers.begin(); it != m_motorTwoDofControllers.end() ; ++it) {
  //   (*it).setup(400.0, 0.04, m_dt);
  //   // (*it).setup(400.0, 1.0, m_dt);
  // }

  // allocate memory for outPorts
  m_qRefOut.data.length(m_robot->numJoints());
  
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
  hrp::dvector qRef;
  qRef.resize(m_robot->numJoints());
  
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
  if (m_qRefInIn.isNew()) {
    m_qRefInIn.read();
  }

  if (m_qRefIn.data.length() == m_robot->numJoints()) {

    // copy buffer
    for (int i = 0; i < m_robot->numJoints(); i++) {
      qRef[i] = m_qRefIn.data[i];
    }
    
    isTempError = limitTempreture(qRef);

    // call beep
    if (isTempError) {
      start_beep(3136);
    } else {
      stop_beep();
    }

    // output restricted qRef
    for (int i = 0; i < m_robot->numJoints(); i++) {
      // if (i == 7) {
      //   m_qRefOut.data[i] = qRef[i];
      // } else {
      //   m_qRefOut.data[i] = m_qRefIn.data[i];
      // }
      m_qRefOut.data[i] = qRef[i];
    }
    m_qRefOut.tm = tm;
    m_qRefOutOut.write();
  }
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

int ThermoLimiter::sgn(double val)
{
  if (val > 0) {
    return 1;
  } else if (val < 0) {
    return -1;
  } else {
    return 0;
  }
}

bool ThermoLimiter::limitTempreture(hrp::dvector &qRef)
{
  static long loop = 0;
  loop++;
  
  int numJoints = m_robot->numJoints();
  bool isTempError = false;
  double temp, tempLimit;
  hrp::dvector squareTauMax(numJoints), distTau(numJoints), dq(numJoints);

  if ( m_tempIn.data.length() ==  m_robot->numJoints() ) {
    if (DEBUGP) {
      std::cerr << "tempreture: ";
      for (int i = 0; i < numJoints; i++) {
        std::cerr << " " << m_tempIn.data[i];
      }
      std::cerr << std::endl;
      std::cerr << "tauIn: ";
      for (int i = 0; i < numJoints; i++) {
        std::cerr << " " << m_tauIn.data[i];
      }
      std::cerr << std::endl;
      std::cerr << "qIn: ";
      for (int i = 0; i < numJoints; i++) {
        std::cerr << " " << m_qRefIn.data[i];
      }
      std::cerr << std::endl;

    }

    for (int i = 0; i < numJoints; i++) {
      temp = m_tempIn.data[i];
      tempLimit = m_motorTempretureLimit[i];
      // check temp limit error
      if (temp > tempLimit){
        isTempError = true;
      }

      // limit tempreture
      // TODO: read coeffs from conf files
      double term = 120;
      squareTauMax[i] = (((tempLimit - temp) / term) + m_motorHeatParams[i].thermoCoeffs * (temp - m_motorHeatParams[i].tempreture)) / m_motorHeatParams[i].currentCoeffs;

      // determine distTau
      if (squareTauMax[i] < 0) {
        std::cerr << "[WARN] tauMax ** 2 = " << squareTauMax[i] << " < 0 in Joint " << i << std::endl;
        distTau[i] = m_tauIn.data[i];
      } else {
        if (std::pow(m_tauIn.data[i], 2) > squareTauMax[i]) {
          std::cerr << "[WARN] tauMax over in Joint " << i << ": " << m_tauIn.data[i] << "^2 > " << squareTauMax[i] << std::endl;
          distTau[i] = sgn(m_tauIn.data[i]) * std::sqrt(squareTauMax[i]);
        } else {
          distTau[i] = m_tauIn.data[i];
        }
      }
      // torque control
      dq[i] = m_motorTwoDofControllers[i].update(m_tauIn.data[i], distTau[i]);
      // qRef[i] += vlimit(dq[i], -DQ_MAX, DQ_MAX);
      qRef[i] -= dq[i]; // twoDofController: tau = -K(q - qRef)
    }
    if (DEBUGP) {
      std::cerr << "tauMax: ";
      for (int i = 0; i < squareTauMax.size(); i++) {
        std::cerr << std::sqrt(squareTauMax[i]) << " ";
      }
      std::cerr << std::endl;
      std::cerr << "distTau: ";
      for (int i = 0; i < distTau.size(); i++) {
        std::cerr << distTau[i] << " ";
      }
      std::cerr << std::endl;
      std::cerr << "dq: ";
      for (int i = 0; i < dq.size(); i++) {
        std::cerr << dq[i] << " ";
      }
      std::cerr << std::endl;
      std::cerr << "qRef: ";
      for (int i = 0; i < qRef.size(); i++) {
        std::cerr << qRef[i] << " ";
      }
      std::cerr << std::endl;
    }
  }
  return isTempError;
}

static double vlimit(double value, double llimit, double ulimit)
{
  if (value > ulimit) {
    return ulimit;
  } else if (value < llimit) {
    return llimit;
  }
  return value;
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

static double vlimit(double value, double llimit, double ulimit);

// Module specification
// <rtc-template block="module_spec">
static const char* thermolimiter_spec[] =
  {
    "implementation_id", "ThermoLimiter",
    "type_name",         "ThermoLimiter",
    "description",       "null component",
    "version",           "1.0",
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
    m_qRefInIn("qRefIn", m_qRefIn),
    m_qCurrentInIn("qCurrentIn", m_qCurrentIn),
    m_qRefOutOut("qRefOut", m_qRefOut),
    m_debugLevel(1)
    // </rtc-template>
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
  // bindParameter("string", confstring, "testtest");
  // bindParameter("intvec", confintvec, "4,5,6,7");
  // bindParameter("double", confdouble, "4.567");
  
  // </rtc-template>

  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("tempIn", m_tempInIn);
  addInPort("tauIn", m_tauInIn);
  addInPort("qCurrentIn", m_qCurrentInIn);
  addInPort("qRefIn", m_qRefInIn);

  // Set OutPort buffer
  addOutPort("qRefOut", m_qRefOutOut);
  
  // Set service provider to Ports
  // m_NullServicePort.registerProvider("service0", "NullService", m_NullService);
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  // addPort(m_NullServicePort);
  
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
  // set limit of motor tempreture
  coil::vstring motorTempretureLimitFromConf = coil::split(prop["motor_tempreture_limit"], ",");
  if (motorTempretureLimitFromConf.size() != m_robot->numJoints()) {
    std::cerr <<  "[WARN]: size of motor_tempreture_limit is " << motorTempretureLimitFromConf.size() << ", not equal to " << m_robot->numJoints() << std::endl;
    m_motorTempretureLimit.resize(m_robot->numJoints(), 100.0);
  } else {
    m_motorTempretureLimit.resize(m_robot->numJoints());
    for (int i = 0; i < m_robot->numJoints(); i++) {
      coil::stringTo(m_motorTempretureLimit[i], motorTempretureLimitFromConf[i].c_str());
    }
  }
  if (m_debugLevel > 0) {
    std::cerr <<  "motor_tempreture_limit: ";
    for(std::vector<double>::iterator it = m_motorTempretureLimit.begin(); it != m_motorTempretureLimit.end(); ++it){
      std::cerr << *it << " ";
    }
    std::cerr << std::endl;
  }

  // set tempreture of environment
  double ambientTemp = 25.0;
  if (prop["ambient_tmp"] != "") {
    coil::stringTo(ambientTemp, prop["ambient_tmp"].c_str());
  }
  std::cerr <<  m_profile.instance_name << ": ambient tempreture: " << ambientTemp << std::endl;

  // set limit of motor heat parameters
  coil::vstring motorHeatParamsFromConf = coil::split(prop["motor_heat_params"], ",");
  m_motorHeatParams.resize(m_robot->numJoints());
  if (motorHeatParamsFromConf.size() != 2 * m_robot->numJoints()) {
    std::cerr <<  "[WARN]: size of motor_heat_param is " << motorHeatParamsFromConf.size() << ", not equal to 2 * " << m_robot->numJoints() << std::endl;
    for (int i = 0; i < m_robot->numJoints(); i++) {
      m_motorHeatParams[i].defaultParams();
      m_motorHeatParams[i].tempreture = ambientTemp;
    }

  } else {
    for (int i = 0; i < m_robot->numJoints(); i++) {
      m_motorHeatParams[i].tempreture = ambientTemp;
      coil::stringTo(m_motorHeatParams[i].currentCoeffs, motorHeatParamsFromConf[2 * i].c_str());
      coil::stringTo(m_motorHeatParams[i].thermoCoeffs, motorHeatParamsFromConf[2 * i + 1].c_str());
    }
  }
  
  if (m_debugLevel > 0) {
    std::cerr <<  "motor_heat_param: ";
    for(std::vector<MotorHeatParam>::iterator it = m_motorHeatParams.begin(); it != m_motorHeatParams.end(); ++it){
      std::cerr << (*it).tempreture << "," << (*it).currentCoeffs << "," << (*it).thermoCoeffs << ", ";
    }
    std::cerr << std::endl;
  }
  
  // make torque controller
  // set limit of motor heat parameters
  coil::vstring torqueControllerParamsFromConf = coil::split(prop["torque_controller_params"], ",");
  m_motorTwoDofControllers.resize(m_robot->numJoints());
  if (torqueControllerParamsFromConf.size() != 2 * m_robot->numJoints()) {
    std::cerr <<  "[WARN]: size of torque_controller_params is " << torqueControllerParamsFromConf.size() << ", not equal to 2 * " << m_robot->numJoints() << std::endl;
    for (std::vector<TwoDofController>::iterator it = m_motorTwoDofControllers.begin(); it != m_motorTwoDofControllers.end() ; ++it) {
      (*it).setup(400.0, 0.04, m_dt); // set default params
      // (*it).setup(400.0, 1.0, m_dt);
    }
  } else {
    double tdcParamK, tdcParamT;
    for (int i = 0; i < m_robot->numJoints(); i++) {
      coil::stringTo(tdcParamK, torqueControllerParamsFromConf[2 * i].c_str());
      coil::stringTo(tdcParamT, torqueControllerParamsFromConf[2 * i + 1].c_str());
      m_motorTwoDofControllers[i].setup(tdcParamK, tdcParamT, m_dt);
    }
  }
  // for (std::vector<TwoDofController>::iterator it = m_motorTwoDofControllers.begin(); it != m_motorTwoDofControllers.end() ; ++it) {
  //   (*it).setup(400.0, 0.04, m_dt);
  //   // (*it).setup(400.0, 1.0, m_dt);
  // }

  // allocate memory for outPorts
  m_qRefOut.data.length(m_robot->numJoints());
  
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
  hrp::dvector qRef;
  qRef.resize(m_robot->numJoints());
  
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
  if (m_qRefInIn.isNew()) {
    m_qRefInIn.read();
  }

  if (m_qRefIn.data.length() == m_robot->numJoints()) {

    // copy buffer
    for (int i = 0; i < m_robot->numJoints(); i++) {
      qRef[i] = m_qRefIn.data[i];
    }
    
    isTempError = limitTempreture(qRef);

    // call beep
    if (isTempError) {
      start_beep(3136);
    } else {
      stop_beep();
    }

    // output restricted qRef
    for (int i = 0; i < m_robot->numJoints(); i++) {
      // if (i == 7) {
      //   m_qRefOut.data[i] = qRef[i];
      // } else {
      //   m_qRefOut.data[i] = m_qRefIn.data[i];
      // }
      m_qRefOut.data[i] = qRef[i];
    }
    m_qRefOut.tm = tm;
    m_qRefOutOut.write();
  }
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

int ThermoLimiter::sgn(double val)
{
  if (val > 0) {
    return 1;
  } else if (val < 0) {
    return -1;
  } else {
    return 0;
  }
}

bool ThermoLimiter::limitTempreture(hrp::dvector &qRef)
{
  static long loop = 0;
  loop++;
  
  int numJoints = m_robot->numJoints();
  bool isTempError = false;
  double temp, tempLimit;
  hrp::dvector squareTauMax(numJoints), distTau(numJoints), dq(numJoints);

  if ( m_tempIn.data.length() ==  m_robot->numJoints() ) {
    if (DEBUGP) {
      std::cerr << "tempreture: ";
      for (int i = 0; i < numJoints; i++) {
        std::cerr << " " << m_tempIn.data[i];
      }
      std::cerr << std::endl;
      std::cerr << "tauIn: ";
      for (int i = 0; i < numJoints; i++) {
        std::cerr << " " << m_tauIn.data[i];
      }
      std::cerr << std::endl;
      std::cerr << "qIn: ";
      for (int i = 0; i < numJoints; i++) {
        std::cerr << " " << m_qRefIn.data[i];
      }
      std::cerr << std::endl;

    }

    for (int i = 0; i < numJoints; i++) {
      temp = m_tempIn.data[i];
      tempLimit = m_motorTempretureLimit[i];
      // check temp limit error
      if (temp > tempLimit){
        isTempError = true;
      }

      // limit tempreture
      // TODO: read coeffs from conf files
      double term = 120;
      squareTauMax[i] = (((tempLimit - temp) / term) + m_motorHeatParams[i].thermoCoeffs * (temp - m_motorHeatParams[i].tempreture)) / m_motorHeatParams[i].currentCoeffs;

      // determine distTau
      if (squareTauMax[i] < 0) {
        std::cerr << "[WARN] tauMax ** 2 = " << squareTauMax[i] << " < 0 in Joint " << i << std::endl;
        distTau[i] = m_tauIn.data[i];
      } else {
        if (std::pow(m_tauIn.data[i], 2) > squareTauMax[i]) {
          std::cerr << "[WARN] tauMax over in Joint " << i << ": " << m_tauIn.data[i] << "^2 > " << squareTauMax[i] << std::endl;
          distTau[i] = sgn(m_tauIn.data[i]) * std::sqrt(squareTauMax[i]);
        } else {
          distTau[i] = m_tauIn.data[i];
        }
      }
      // torque control
      dq[i] = m_motorTwoDofControllers[i].update(m_tauIn.data[i], distTau[i]);
      // qRef[i] += vlimit(dq[i], -DQ_MAX, DQ_MAX);
      qRef[i] -= dq[i]; // twoDofController: tau = -K(q - qRef)
    }
    if (DEBUGP) {
      std::cerr << "tauMax: ";
      for (int i = 0; i < squareTauMax.size(); i++) {
        std::cerr << std::sqrt(squareTauMax[i]) << " ";
      }
      std::cerr << std::endl;
      std::cerr << "distTau: ";
      for (int i = 0; i < distTau.size(); i++) {
        std::cerr << distTau[i] << " ";
      }
      std::cerr << std::endl;
      std::cerr << "dq: ";
      for (int i = 0; i < dq.size(); i++) {
        std::cerr << dq[i] << " ";
      }
      std::cerr << std::endl;
      std::cerr << "qRef: ";
      for (int i = 0; i < qRef.size(); i++) {
        std::cerr << qRef[i] << " ";
      }
      std::cerr << std::endl;
    }
  }
  return isTempError;
}

static double vlimit(double value, double llimit, double ulimit)
{
  if (value > ulimit) {
    return ulimit;
  } else if (value < llimit) {
    return llimit;
  }
  return value;
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


