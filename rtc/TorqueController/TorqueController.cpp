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

#include <map>

// Module specification
// <rtc-template block="module_spec">
static const char* torquecontroller_spec[] =
{
  "implementation_id", "TorqueController",
  "type_name",         "TorqueController",
  "description",       "Component for joint torque control",
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
    m_debugLevel(0)
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
  bindParameter("debugLevel", m_debugLevel, "0");
  
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
  // make controlle type map
  MotorTorqueController::motor_model_t model_type; 
  std::map<int, MotorTorqueController::motor_model_t> param_num_to_motor_model_type;
  int tdc_params_num = TwoDofController::TwoDofControllerParam::getControllerParamNum() * m_robot->numJoints();
  int tdc_pd_model_params_num = TwoDofControllerPDModel::TwoDofControllerPDModelParam::getControllerParamNum() * m_robot->numJoints();
  int tdc_dynamics_model_params_num = TwoDofControllerDynamicsModel::TwoDofControllerDynamicsModelParam::getControllerParamNum() * m_robot->numJoints();
  param_num_to_motor_model_type[tdc_params_num] = MotorTorqueController::TWO_DOF_CONTROLLER;
  param_num_to_motor_model_type[tdc_pd_model_params_num] = MotorTorqueController::TWO_DOF_CONTROLLER_PD_MODEL;
  param_num_to_motor_model_type[tdc_dynamics_model_params_num] = MotorTorqueController::TWO_DOF_CONTROLLER_DYNAMICS_MODEL;
  if (param_num_to_motor_model_type.find(motorTorqueControllerParamsFromConf.size()) == param_num_to_motor_model_type.end()) {
      std::cerr << "[" <<  m_profile.instance_name << "]" << "torque_controller_params is not correct number, " << motorTorqueControllerParamsFromConf.size() << ". Use default controller." << std::endl;
    model_type = MotorTorqueController::TWO_DOF_CONTROLLER; // default
  } else {
    model_type = param_num_to_motor_model_type[motorTorqueControllerParamsFromConf.size()];
  }
  // define controller paramters
  switch (model_type) {
  case MotorTorqueController::TWO_DOF_CONTROLLER_DYNAMICS_MODEL: // use TwoDofControllerDynamicsModel
  { // limit scope of tdc_dynamics_model_params
    std::cerr << "[" <<  m_profile.instance_name << "]" << "use TwoDofControllerDynamicsModel" << std::endl;
    std::vector<TwoDofControllerDynamicsModel::TwoDofControllerDynamicsModelParam> tdc_dynamics_model_params(m_robot->numJoints());
    for (int i = 0; i < m_robot->numJoints(); i++) { 
      if (motorTorqueControllerParamsFromConf.size() == tdc_dynamics_model_params_num) { // use conf params if parameter num is correct
        coil::stringTo(tdc_dynamics_model_params[i].alpha, motorTorqueControllerParamsFromConf[4 * i].c_str());
        coil::stringTo(tdc_dynamics_model_params[i].beta, motorTorqueControllerParamsFromConf[4 * i + 1].c_str());
        coil::stringTo(tdc_dynamics_model_params[i].ki, motorTorqueControllerParamsFromConf[4 * i + 2].c_str());
        coil::stringTo(tdc_dynamics_model_params[i].tc, motorTorqueControllerParamsFromConf[4 * i + 3].c_str());
      }
      tdc_dynamics_model_params[i].dt = m_dt;
      m_motorTorqueControllers.push_back(MotorTorqueController(m_robot->joint(i)->name, tdc_dynamics_model_params[i]));
    }
    if (m_debugLevel > 0) {
      std::cerr << "[" <<  m_profile.instance_name << "]" << "torque controller parames:" << std::endl;
      for (int i = 0; i < m_robot->numJoints(); i++) {
        std::cerr << "[" <<  m_profile.instance_name << "]" << m_robot->joint(i)->name << ":"
                  << tdc_dynamics_model_params[i].alpha << " " << tdc_dynamics_model_params[i].beta << " " << tdc_dynamics_model_params[i].ki
                  << " " << tdc_dynamics_model_params[i].tc << " " << tdc_dynamics_model_params[i].dt << std::endl;
      }
    }
    break;
  }
  case MotorTorqueController::TWO_DOF_CONTROLLER_PD_MODEL: // use TwoDofControllerPDModel
  { // limit scope of tdc_pd_model_params
    std::cerr << "[" <<  m_profile.instance_name << "]" << "use TwoDofControllerPDModel" << std::endl;
    std::vector<TwoDofControllerPDModel::TwoDofControllerPDModelParam> tdc_pd_model_params(m_robot->numJoints());
    for (int i = 0; i < m_robot->numJoints(); i++) {
      if (motorTorqueControllerParamsFromConf.size() == tdc_pd_model_params_num) { // use conf params if parameter num is correct
        coil::stringTo(tdc_pd_model_params[i].ke, motorTorqueControllerParamsFromConf[3 * i].c_str());
        coil::stringTo(tdc_pd_model_params[i].kd, motorTorqueControllerParamsFromConf[3 * i + 1].c_str());
        coil::stringTo(tdc_pd_model_params[i].tc, motorTorqueControllerParamsFromConf[3 * i + 2].c_str());
      }
      tdc_pd_model_params[i].dt = m_dt;
      m_motorTorqueControllers.push_back(MotorTorqueController(m_robot->joint(i)->name, tdc_pd_model_params[i]));
    }
    if (m_debugLevel > 0) {
      std::cerr << "[" <<  m_profile.instance_name << "]" << "torque controller parames:" << std::endl;
      for (int i = 0; i < m_robot->numJoints(); i++) {
        std::cerr << "[" <<  m_profile.instance_name << "]" << m_robot->joint(i)->name << ":"
                  << tdc_pd_model_params[i].ke << " " << tdc_pd_model_params[i].kd
                  << " " << tdc_pd_model_params[i].tc << " " << tdc_pd_model_params[i].dt << std::endl;
      }
    }
    break;
  }
  case MotorTorqueController::TWO_DOF_CONTROLLER: // use TwoDofController
  default:
  { // limit scope of tdc_params
    std::cerr << "[" <<  m_profile.instance_name << "]" << "use TwoDofController" << std::endl;
    std::vector<TwoDofController::TwoDofControllerParam> tdc_params(m_robot->numJoints());
    for (int i = 0; i < m_robot->numJoints(); i++) { 
      if (motorTorqueControllerParamsFromConf.size() == tdc_params_num) { // use conf params if parameter num is correct
        coil::stringTo(tdc_params[i].ke, motorTorqueControllerParamsFromConf[2 * i].c_str());
        coil::stringTo(tdc_params[i].tc, motorTorqueControllerParamsFromConf[2 * i + 1].c_str());
      }
      tdc_params[i].dt = m_dt;
      m_motorTorqueControllers.push_back(MotorTorqueController(m_robot->joint(i)->name, tdc_params[i]));
    }
    if (m_debugLevel > 0) {
      std::cerr << "[" <<  m_profile.instance_name << "]" << "torque controller parames:" << std::endl;
      for (int i = 0; i < m_robot->numJoints(); i++) {
        std::cerr << "[" <<  m_profile.instance_name << "]" << m_robot->joint(i)->name << ":"
                  << tdc_params[i].ke << " " << tdc_params[i].tc << " " << tdc_params[i].dt << std::endl;
      }
    }
    break;
  }
  }

  // parameter setttings for torque controller
  for (int i = 0; i < m_robot->numJoints(); i++) {
    m_motorTorqueControllers[i].setErrorPrefix(std::string(m_profile.instance_name));
    m_motorTorqueControllers[i].setupMotorControllerMinMaxDq(m_robot->joint(i)->lvlimit * m_dt, m_robot->joint(i)->uvlimit * m_dt);
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
  m_loop++;

  hrp::dvector dq(m_robot->numJoints());
  
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
      m_tauCurrentIn.data.length() == m_robot->numJoints() &&
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
      m_qRefOut.data[i] = std::min(std::max(m_qRefIn.data[i] + dq[i], m_robot->joint(i)->llimit), m_robot->joint(i)->ulimit);
    }
  } else {
    if (isDebug()) {
      std::cerr << "[" <<  m_profile.instance_name << "]" << "TorqueController input is not correct" << std::endl;
      std::cerr << "[" <<  m_profile.instance_name << "]" << " numJoints: " << m_robot->numJoints() << std::endl;
      std::cerr << "[" <<  m_profile.instance_name << "]" << "  qCurrent: " << m_qCurrentIn.data.length() << std::endl;
      std::cerr << "[" <<  m_profile.instance_name << "]" << "    qRefIn: " << m_qRefIn.data.length() << std::endl;
      std::cerr << "[" <<  m_profile.instance_name << "]" << "tauCurrent: " << m_tauCurrentIn.data.length() << std::endl;
      std::cerr << std::endl;
    }
    // pass qRefIn to qRefOut
    for (int i = 0; i < m_robot->numJoints(); i++) {
      m_qRefOut.data[i] = m_qRefIn.data[i];
    }
  }

  m_qRefOut.tm = m_qRefIn.tm;
  m_qRefOutOut.write();

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
  int numJoints = m_robot->numJoints();
  hrp::dvector tauMax(numJoints);
  dq.resize(numJoints);

  // determine tauMax
  for(int i = 0; i < numJoints; i++) {
    double tauMaxFromModel = m_robot->joint(i)->climit * m_robot->joint(i)->gearRatio * m_robot->joint(i)->torqueConst;
    if ( m_tauMaxIn.data.length() ==  m_robot->numJoints() ) {
      tauMax[i] = std::min(tauMaxFromModel, m_tauMaxIn.data[i]);
    } else {
      tauMax[i] = tauMaxFromModel;
    }
  }

  // execute torque control
  // tauCurrent.length is assumed to be equal to numJoints (check in onExecute)
  if (isDebug()) {
    std::cerr << "[" <<  m_profile.instance_name << "]" << "tauCurrentIn: ";
    for (int i = 0; i < numJoints; i++) {
      std::cerr << " " << m_tauCurrentIn.data[i];
    }
    std::cerr << std::endl;
    std::cerr << "[" <<  m_profile.instance_name << "]" << "tauMax: ";
    for (int i = 0; i < numJoints; i++) {
      std::cerr << " " << tauMax[i];
    }
    std::cerr << std::endl;
  }

  Guard guard(m_mutex);
  for (int i = 0; i < numJoints; i++) {
    dq[i] = m_motorTorqueControllers[i].execute(m_tauCurrentIn.data[i], tauMax[i]); // twoDofController: tau = -K(q - qRef)
    // output debug message
    if (isDebug() && m_motorTorqueControllers[i].getMotorControllerState() != MotorTorqueController::INACTIVE) {
      m_motorTorqueControllers[i].printMotorControllerVariables();
    }

  }
    
  if (isDebug()) {
    std::cerr << "[" <<  m_profile.instance_name << "]" << "dq: ";
    for (int i = 0; i < dq.size(); i++) {
      std::cerr << dq[i] << " ";
    }
    std::cerr << std::endl;
  }

  return;
}

bool TorqueController::enableTorqueController(std::string jname)
{
  bool succeed = false;
  for (std::vector<MotorTorqueController>::iterator it = m_motorTorqueControllers.begin(); it != m_motorTorqueControllers.end(); ++it) {
    if ((*it).getJointName() == jname){
      if (m_debugLevel > 0) {
        std::cerr << "[" <<  m_profile.instance_name << "]" << "Enable torque controller in " << jname << std::endl;
      }
      succeed = (*it).enable();
    }
  }
  return succeed;
}

bool TorqueController::enableMultipleTorqueControllers(const OpenHRP::TorqueControllerService::StrSequence& jnames)
{
  bool succeed = true;
  bool retval;
  for (int i = 0; i < jnames.length(); i++) {
    retval = enableTorqueController(std::string(jnames[i]));
    if (!retval) { // return false when once failed
      succeed = false;
    }
  }
  return succeed;
}

bool TorqueController::disableTorqueController(std::string jname)
{
  bool succeed = false;
  for (std::vector<MotorTorqueController>::iterator it = m_motorTorqueControllers.begin(); it != m_motorTorqueControllers.end(); ++it) {
    if ((*it).getJointName() == jname){
      if (m_debugLevel > 0) {
        std::cerr << "[" <<  m_profile.instance_name << "]" << "Disable torque controller in " << jname << std::endl;
      }
      succeed = (*it).disable();
    }
  }
  return succeed;
}

bool TorqueController::disableMultipleTorqueControllers(const OpenHRP::TorqueControllerService::StrSequence& jnames)
{
  bool succeed = true;
  bool retval;
  for (int i = 0; i < jnames.length(); i++) {
    retval = disableTorqueController(std::string(jnames[i]));
    if (!retval) { // return false when once failed
      succeed = false;
    }
  }
  return succeed;
}

bool TorqueController::startTorqueControl(std::string jname)
{
  bool succeed = false;
  for (std::vector<MotorTorqueController>::iterator it = m_motorTorqueControllers.begin(); it != m_motorTorqueControllers.end(); ++it) {
    if ((*it).getJointName() == jname){
      if (m_debugLevel > 0) {
        std::cerr << "[" <<  m_profile.instance_name << "]" << "Start torque control in " << jname << std::endl;
      }
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
  return succeed;
}

bool TorqueController::stopTorqueControl(std::string jname)
{
  bool succeed = false;
  for (std::vector<MotorTorqueController>::iterator it = m_motorTorqueControllers.begin(); it != m_motorTorqueControllers.end(); ++it) {
    if ((*it).getJointName() == jname){
      if (m_debugLevel > 0) {
        std::cerr << "[" <<  m_profile.instance_name << "]" << "Stop torque control in " << jname << std::endl;
      }
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
  return succeed;
}

bool TorqueController::setReferenceTorque(std::string jname, double tauRef)
{
  bool succeed = false;
  
  // lock mutex
  Guard guard(m_mutex);

  // search target joint
  for (std::vector<MotorTorqueController>::iterator it = m_motorTorqueControllers.begin(); it != m_motorTorqueControllers.end(); ++it) {
    if ((*it).getJointName() == jname) {
      if (m_debugLevel > 0) {
        std::cerr << "[" <<  m_profile.instance_name << "]" << "Set " << jname << " reference torque to " << tauRef << std::endl;
      }
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
    std::cerr << "[" <<  m_profile.instance_name << "]" << "Length of jnames and tauRefs are different." << std::endl;
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

bool TorqueController::setTorqueControllerParam(const OpenHRP::TorqueControllerService::torqueControllerParam& t_param)
{
  // find target motor controller
  std::string jname = std::string(t_param.name);
  MotorTorqueController *tgt_controller = NULL;
  for (std::vector<MotorTorqueController>::iterator it = m_motorTorqueControllers.begin(); it != m_motorTorqueControllers.end(); ++it) {
    if ((*it).getJointName() == jname){
      std::cerr << "[" <<  m_profile.instance_name << "]" << "target joint:" << t_param.name << std::endl;
      tgt_controller = &(*it);
    }
  }
  if (tgt_controller == NULL) {
    std::cerr << "[" <<  m_profile.instance_name << "]" << t_param.name << "does not found." << std::endl;
    return false;
  }

  // update torque controller param
  bool retval;
  MotorTorqueController::motor_model_t model_type = tgt_controller->getMotorModelType();
  switch(model_type) { // dt is defined by controller cycle
  case MotorTorqueController::TWO_DOF_CONTROLLER:
  { // limit scope for param 
    std::cerr << "[" <<  m_profile.instance_name << "]" << "new param:" << t_param.ke << " " << t_param.tc << " " << std::endl;
    TwoDofController::TwoDofControllerParam param;
    param.ke = t_param.ke; param.tc = t_param.tc; param.dt = m_dt;
    retval = tgt_controller->updateControllerParam(param);
    break;
  }
  case MotorTorqueController::TWO_DOF_CONTROLLER_PD_MODEL:
  { // limit scope for param 
    std::cerr << "[" <<  m_profile.instance_name << "]" << "new param:" << t_param.ke << " " << t_param.kd << " " << t_param.tc << " " << std::endl;
    TwoDofControllerPDModel::TwoDofControllerPDModelParam param;
    param.ke = t_param.ke; param.kd = t_param.kd; param.tc = t_param.tc; param.dt = m_dt;
    retval = tgt_controller->updateControllerParam(param);
    break;
  }
  case MotorTorqueController::TWO_DOF_CONTROLLER_DYNAMICS_MODEL:
  { // limit scope for param 
    std::cerr << "[" <<  m_profile.instance_name << "]" << "new param:" << t_param.alpha << " " << t_param.beta << " " << t_param.ki << " " << t_param.tc << " " << std::endl;
    TwoDofControllerDynamicsModel::TwoDofControllerDynamicsModelParam param;
    param.alpha = t_param.alpha; param.beta = t_param.beta; param.ki = t_param.ki; param.tc = t_param.tc; param.dt = m_dt;
    retval = tgt_controller->updateControllerParam(param);
    break;
  }
  default:
    return false;
  }
  
  return retval;
}

void TorqueController::updateParam(double &val, double &val_new)
{
  // update value unless val_new is not 0
  if (val_new != 0) { 
    val = val_new;
  }
  return;
}

bool TorqueController::isDebug(int cycle)
{
  return ((m_debugLevel == 1 && (m_loop % cycle == 0)) || m_debugLevel > 1);
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


