// -*- C++ -*-
/*!
 * @file  TorqueController.cpp
 * @brief null component
 * $Date$
 *
 * $Id$
 */

#include "TorqueController.h"
#include "util/VectorConvert.h"

#include <rtm/CorbaNaming.h>
#include <hrpModel/ModelLoaderUtil.h>
#include <hrpUtil/MatrixSolvers.h>

// Module specification
// <rtc-template block="module_spec">
static const char* torquecontroller_spec[] =
{
  "implementation_id", "TorqueController",
  "type_name",         "TorqueController",
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

#define DEBUGP ((m_debugLevel==1 && loop%200==0) || m_debugLevel > 1 )
#define DQ_MAX 1.0

typedef coil::Guard<coil::Mutex> Guard;

TorqueController::TorqueController(RTC::Manager* manager)
  : RTC::DataFlowComponentBase(manager),
    // <rtc-template block="initializer">
    m_tauCurrentInIn("tauCurrent", m_tauCurrentIn),
    m_tauMaxInIn("tauMax", m_tauMaxIn),
    m_qRefInIn("qRef", m_qRefIn),
    m_qCurrentInIn("qCurrent", m_qCurrentIn),
    m_qRefOutOut("q", m_qRefOut),
    m_TorqueControllerServicePort("TorqueControllerService"),
	m_debugLevel(1)
{
  m_service0.torque_controller(this);
}

TorqueController::~TorqueController()
{
}



RTC::ReturnCode_t TorqueController::onInitialize()
{
  std::cout << m_profile.instance_name << ": onInitialize()" << std::endl;
  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  bindParameter("debugLevel", m_debugLevel, "1");
  
  // </rtc-template>

  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("tauCurrent", m_tauCurrentInIn);
  addInPort("tauMax", m_tauMaxInIn);
  addInPort("qCurrent", m_qCurrentInIn);
  addInPort("qRef", m_qRefInIn); // for naming rule of hrpsys_config.py

  // Set OutPort buffer
  addOutPort("q", m_qRefOutOut); // for naming rule of hrpsys_config.py
  
  // Set service provider to Ports
  m_TorqueControllerServicePort.registerProvider("service0", "TorqueControllerService", m_service0);
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  addPort(m_TorqueControllerServicePort);
  
  // </rtc-template>

  // read property settings
  RTC::Properties& prop = getProperties();
  // get dt
  coil::stringTo(m_dt, prop["dt"].c_str()); 
  // make rtc manager settings
  RTC::Manager& rtcManager = RTC::Manager::instance();
  std::string nameServer = rtcManager.getConfig()["corba.nameservers"];
  int comPos = nameServer.find(",");
  if (comPos < 0){
    comPos = nameServer.length();
  }
  nameServer = nameServer.substr(0, comPos);
  RTC::CorbaNaming naming(rtcManager.getORB(), nameServer.c_str());
  // set robot model
  m_robot = hrp::BodyPtr(new hrp::Body());
  std::cerr << prop["model"].c_str() << std::endl;
  if (!loadBodyFromModelLoader(m_robot, prop["model"].c_str(),
                               CosNaming::NamingContext::_duplicate(naming.getRootContext())
        )){
    std::cerr << "failed to load model[" << prop["model"] << "]"
              << std::endl;
  }
  // make torque controller settings
  coil::vstring motorTorqueControllerParamsFromConf = coil::split(prop["torque_controller_params"], ",");
 
  hrp::dvector tdcParamK(m_robot->numJoints()), tdcParamT(m_robot->numJoints());
  if (motorTorqueControllerParamsFromConf.size() == 2 * m_robot->numJoints()) {
    for (int i = 0; i < m_robot->numJoints(); i++) {
      coil::stringTo(tdcParamK[i], motorTorqueControllerParamsFromConf[2 * i].c_str());
      coil::stringTo(tdcParamT[i], motorTorqueControllerParamsFromConf[2 * i + 1].c_str());
      m_motorTorqueControllers.push_back(MotorTorqueController(m_robot->joint(i)->name, tdcParamK[i], tdcParamT[i], m_dt));
    }
  } else { // default
    std::cerr << "[WARNING] torque_controller_params is not correct number, " << motorTorqueControllerParamsFromConf.size() << std::endl;
    for (int i = 0; i < m_robot->numJoints(); i++) {
      tdcParamK[i] = 400.0;
      tdcParamT[i] = 0.04;
      m_motorTorqueControllers.push_back(MotorTorqueController(m_robot->joint(i)->name, tdcParamK[i], tdcParamT[i], m_dt));
    }
  }

  if (m_debugLevel > 0) {
    std::cerr << "torque controller parames:" << std::endl;
    for (int i = 0; i < m_robot->numJoints(); i++) {
      std::cerr << m_robot->joint(i)->name << ":" << tdcParamK[i] << " " << tdcParamT[i] << " " << m_dt << std::endl;
    }
  }


  // allocate memory for outPorts
  m_qRefOut.data.length(m_robot->numJoints());
  return RTC::RTC_OK;
}



/*
RTC::ReturnCode_t TorqueController::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t TorqueController::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t TorqueController::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

RTC::ReturnCode_t TorqueController::onActivated(RTC::UniqueId ec_id)
{
  std::cout << m_profile.instance_name<< ": onActivated(" << ec_id << ")" << std::endl;
  return RTC::RTC_OK;
}

RTC::ReturnCode_t TorqueController::onDeactivated(RTC::UniqueId ec_id)
{
  std::cout << m_profile.instance_name<< ": onDeactivated(" << ec_id << ")" << std::endl;
  return RTC::RTC_OK;
}

RTC::ReturnCode_t TorqueController::onExecute(RTC::UniqueId ec_id)
{ 
  // make timestamp
  coil::TimeValue coiltm(coil::gettimeofday());
  hrp::dvector dq(m_robot->numJoints());
  RTC::Time tm;
  tm.sec = coiltm.sec();
  tm.nsec = coiltm.usec()*1000;
  
  // update port
  if (m_tauCurrentInIn.isNew()) {
    m_tauCurrentInIn.read();
  }
  if (m_tauMaxInIn.isNew()) {
    m_tauMaxInIn.read();
  }
  if (m_qCurrentInIn.isNew()) {
    m_qCurrentInIn.read();
  }
  if (m_qRefInIn.isNew()) {
    m_qRefInIn.read();
  }

  if (m_qRefIn.data.length() == m_robot->numJoints() &&
      m_qCurrentIn.data.length() == m_robot->numJoints()) {

    // update model
    for ( int i = 0; i < m_robot->numJoints(); i++ ){
        m_robot->joint(i)->q = m_qCurrentIn.data[i];
    }
    m_robot->calcForwardKinematics();   

    // calculate dq by torque controller
    executeTorqueControl(dq);

    // output restricted qRef
    for (int i = 0; i < m_robot->numJoints(); i++) {
      m_qRefOut.data[i] = std::min(std::max(m_qRefIn.data[i] - dq[i], m_robot->joint(i)->llimit), m_robot->joint(i)->ulimit);      
    }
    m_qRefOut.tm = tm;
    m_qRefOutOut.write();
  } else {
    std::cerr << "TorqueController input is not correct" << std::endl;
    std::cerr << "qCurrent: " << m_qCurrentIn.data.length() << std::endl;
    std::cerr << "qRef: " << m_qRefIn.data.length() << std::endl;
    std::cerr << std::endl;
  }
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t TorqueController::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t TorqueController::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t TorqueController::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t TorqueController::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t TorqueController::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

void TorqueController::executeTorqueControl(hrp::dvector &dq)
{
  static int loop = 0;
  int numJoints = m_robot->numJoints();
  hrp::dvector tauMax(numJoints);
  loop++;
  dq.resize(numJoints);

  if (DEBUGP) {
    std::cerr << "[" << m_profile.instance_name << "]" << std::endl;
  }
  
  // determine tauMax
  for(int i = 0; i < numJoints; i++){
    if ( m_tauMaxIn.data.length() ==  m_robot->numJoints() ) {
      tauMax[i] = std::min(m_robot->joint(i)->climit, m_tauMaxIn.data[i]);
    } else {
      tauMax[i] = m_robot->joint(i)->climit;
    }
  }

  // execute torque control
  if ( m_tauCurrentIn.data.length() ==  m_robot->numJoints() ) {
    if (DEBUGP) {
      std::cerr << "tauCurrentIn: ";
      for (int i = 0; i < numJoints; i++) {
        std::cerr << " " << m_tauCurrentIn.data[i];
      }
      std::cerr << std::endl;
      std::cerr << "tauMax: ";
      for (int i = 0; i < numJoints; i++) {
        std::cerr << " " << tauMax[i];
      }
      std::cerr << std::endl;
    }

    Guard guard(m_mutex);
    for (int i = 0; i < numJoints; i++) {
      dq[i] = m_motorTorqueControllers[i].execute(m_tauCurrentIn.data[i], tauMax[i]); // twoDofController: tau = -K(q - qRef)
      // output debug message
      if (DEBUGP && m_motorTorqueControllers[i].getMotorControllerState() != MotorTorqueController::INACTIVE) {
        m_motorTorqueControllers[i].printMotorControllerVariables();
      }

    }
    
    if (DEBUGP) {
      std::cerr << "dq: ";
      for (int i = 0; i < dq.size(); i++) {
        std::cerr << dq[i] << " ";
      }
      std::cerr << std::endl;
    }
  } else {
    std::cerr << "TorqueController input is not correct" << std::endl;
    std::cerr << "tauCurrent: " << m_tauCurrentIn.data.length() << std::endl;
    std::cerr << std::endl;
  }

  return;
}

bool TorqueController::startTorqueControl(std::string jname)
{
  bool succeed = false;
  for (std::vector<MotorTorqueController>::iterator it = m_motorTorqueControllers.begin(); it != m_motorTorqueControllers.end(); ++it) {
    if ((*it).getJointName() == jname){
      std::cerr << "Start torque control in " << jname << std::endl;
      succeed = (*it).activate();
    }
  }
  return succeed;
}

bool TorqueController::startMultipleTorqueControls(const OpenHRP::TorqueControllerService::StrSequence& jnames)
{
  bool succeed = true;
  bool retval;
  for (int i = 0; i < jnames.length(); i++) {
    retval = startTorqueControl(std::string(jnames[i]));
    if (!retval) { // return false when once failed
      succeed = false;
    }
  }
}

bool TorqueController::stopTorqueControl(std::string jname)
{
  bool succeed = false;
  for (std::vector<MotorTorqueController>::iterator it = m_motorTorqueControllers.begin(); it != m_motorTorqueControllers.end(); ++it) {
    if ((*it).getJointName() == jname){
      std::cerr << "Stop torque control in " << jname << std::endl;
      succeed = (*it).deactivate();
    }
  }
  return succeed;
}

bool TorqueController::stopMultipleTorqueControls(const OpenHRP::TorqueControllerService::StrSequence& jnames)
{
  bool succeed = true;
  bool retval;
  for (int i = 0; i < jnames.length(); i++) {
    retval = stopTorqueControl(std::string(jnames[i]));
    if (!retval) { // return false when once failed
      succeed = false;
    }
  }
}

bool TorqueController::setReferenceTorque(std::string jname, double tauRef)
{
  bool succeed = false;
  
  // Lock Mutex
  Guard guard(m_mutex);

  // Search target joint
  for (std::vector<MotorTorqueController>::iterator it = m_motorTorqueControllers.begin(); it != m_motorTorqueControllers.end(); ++it) {
    if ((*it).getJointName() == jname){
      std::cerr << "Set " << jname << " reference torque to " << tauRef << std::endl;
      succeed = (*it).setReferenceTorque(tauRef);
    }
  }
  return succeed;
}

bool TorqueController::setMultipleReferenceTorques(const OpenHRP::TorqueControllerService::StrSequence& jnames, const OpenHRP::TorqueControllerService::dSequence& tauRefs)
{
  bool succeed = true;
  bool retval;
  // check accordance of joint name and tauRefs
  if (jnames.length() != tauRefs.length()) {
    std::cerr << "[ERROR] Length of jnames and tauRefs are different." << std::endl;
    return false;
  }
  // set reference torques
  for (int i = 0; i < jnames.length(); i++) {
    retval = setReferenceTorque(std::string(jnames[i]), tauRefs[i]);
    if (!retval) { // return false when once failed
      succeed = false;
    }
  }
  return succeed;
}

extern "C"
{

  void TorqueControllerInit(RTC::Manager* manager)
  {
    RTC::Properties profile(torquecontroller_spec);
    manager->registerFactory(profile,
                             RTC::Create<TorqueController>,
                             RTC::Delete<TorqueController>);
  }

};


