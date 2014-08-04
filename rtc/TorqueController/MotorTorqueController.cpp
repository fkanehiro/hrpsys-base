// -*- C++ -*-

/*!
 * @file  MotorTorqueController.cpp
 * @brief torque controller for single motor
 * @date  
 *
 * $Id$
 */

#include "MotorTorqueController.h"
// #include "util/Hrpsys.h"
#include <iostream>
#include <cmath>
#include <boost/math/special_functions/sign.hpp>

#define TRANSITION_TIME 2.0 // [sec]
#define MAX_TRANSITION_COUNT (TRANSITION_TIME/m_dt)
#define TORQUE_MARGIN 10.0 // [Nm]
#define DEFAULT_MIN_MAX_DQ (0.17 * m_dt) // default min/max is 10[deg/sec] = 0.17[rad/sec]

MotorTorqueController::MotorTorqueController()
{
  // default constructor: _jname = "", _ke = _tc = _dt = 0.0
  setupController(0.0, 0.0, 0.0);
  setupControllerCommon("", 0.0);
  setupMotorControllerMinMaxDq(0.0, 0.0);
}

MotorTorqueController::MotorTorqueController(std::string _jname, double _ke, double _tc, double _dt)
{
  setupController(_ke, _tc, _dt);
  setupControllerCommon(_jname, _dt);
  setupMotorControllerMinMaxDq(-DEFAULT_MIN_MAX_DQ, DEFAULT_MIN_MAX_DQ); 
}

MotorTorqueController::MotorTorqueController(std::string _jname, double _ke, double _kd, double _tc, double _dt)
{
  setupController(_ke, _kd, _tc, _dt);
  setupControllerCommon(_jname, _dt);
  setupMotorControllerMinMaxDq(-DEFAULT_MIN_MAX_DQ, DEFAULT_MIN_MAX_DQ);
}

MotorTorqueController::MotorTorqueController(std::string _jname, double _alpha, double _beta, double _ki, double _tc, double _dt)
{
  setupController(_alpha, _beta, _ki, _tc, _dt);
  setupControllerCommon(_jname, _dt);
  setupMotorControllerMinMaxDq(-DEFAULT_MIN_MAX_DQ, DEFAULT_MIN_MAX_DQ);
}

MotorTorqueController::~MotorTorqueController(void)
{
}

void MotorTorqueController::setupController(double _ke, double _tc, double _dt)
{
  m_normalController.setupTwoDofController(_ke, _tc, _dt);
  m_emergencyController.setupTwoDofController(_ke, _tc, _dt);
}

void MotorTorqueController::setupController(double _ke, double _kd, double _tc, double _dt)
{
  m_normalController.setupTwoDofControllerPDModel(_ke, _kd, _tc, _dt);
  m_emergencyController.setupTwoDofControllerPDModel(_ke, _kd, _tc, _dt);
}

void MotorTorqueController::setupController(double _alpha, double _beta, double _ki, double _tc, double _dt)
{
  m_normalController.setupTwoDofControllerDynamicsModel(_alpha, _beta, _ki, _tc, _dt);
  m_emergencyController.setupTwoDofControllerDynamicsModel(_alpha, _beta, _ki, _tc, _dt);
}

void MotorTorqueController::setupMotorControllerMinMaxDq(double _min_dq, double _max_dq)
{
  m_normalController.min_dq = _min_dq;
  m_emergencyController.min_dq = _min_dq;
  m_normalController.max_dq = _max_dq;
  m_emergencyController.max_dq = _max_dq;
  return;
}

bool MotorTorqueController::activate(void)
{
  bool retval = false;
  if (m_normalController.state == INACTIVE) {
    resetMotorControllerVariables(m_normalController);
    m_normalController.controller->reset();
    m_normalController.state = ACTIVE;
    retval = true;
  } else {
    std::cerr << "[ERROR] Torque control in " << m_joint_name << " is already active" << std::endl;
    retval = false;
  }
  return retval;
}

bool MotorTorqueController::deactivate(void)
{
  prepareStop(m_normalController);
  return true;
}

bool MotorTorqueController::setReferenceTorque(double _tauRef)
{
  m_command_tauRef = _tauRef;
  return true;
}

double MotorTorqueController::execute (double _tau, double _tauMax)
{
  // define controller state
  double dq, limitedTauRef;
 
  // define emergency state
  if (std::abs(_tau) > std::abs(_tauMax)) {
    if (m_emergencyController.state != ACTIVE) {
      // save transtion of current controller 
      if (m_emergencyController.state != INACTIVE) {
        m_emergencyController.transition_dq = m_emergencyController.getMotorControllerDq();
      } else if (m_normalController.state != INACTIVE) {
        m_emergencyController.transition_dq = m_normalController.getMotorControllerDq();
      }
      m_emergencyController.dq = 0;
      m_emergencyController.controller->reset();
      m_emergencyController.state = ACTIVE;
    }
  } else {
    if (m_emergencyController.state == ACTIVE &&
        std::abs(_tau) <= std::max(std::abs(_tauMax) - TORQUE_MARGIN, 0.0)) {
      if (m_normalController.state != INACTIVE) { // take control over normal process
        m_normalController.transition_dq = m_emergencyController.getMotorControllerDq();
        m_emergencyController.state = INACTIVE;
      } else { // activate stop process for emergency
        prepareStop(m_emergencyController);
      }
    }
  }

  // execute torque control and renew state
  limitedTauRef = std::min(std::max(-_tauMax, m_command_tauRef), _tauMax);
  updateController(_tau, limitedTauRef, m_normalController);
  dq = m_normalController.getMotorControllerDq();
  if (m_emergencyController.state != INACTIVE) { // overwrite by tauMax control when emergency mode
    limitedTauRef = boost::math::copysign(_tauMax, _tau);
    updateController(_tau, limitedTauRef, m_emergencyController);
    dq = m_emergencyController.getMotorControllerDq();
  }

  // for debug
  m_current_tau = _tau;
  m_actual_tauRef = limitedTauRef;
  
  return dq;
}

std::string MotorTorqueController::getJointName(void)
{
  return m_joint_name;
}

MotorTorqueController::controller_state_t MotorTorqueController::getMotorControllerState(void)
{
  if (m_emergencyController.state == INACTIVE) {
    return m_normalController.state;
  } else {
    return m_emergencyController.state;
  }
}

void MotorTorqueController::printMotorControllerVariables(void)
{
  std::string prefix = "[MotorTorqueController]";
  prefix += m_joint_name + ".";
  std::cerr << prefix << "normalController.state:" << m_normalController.state  << std::endl;
  std::cerr << prefix << "normalController.dq:" << m_normalController.getMotorControllerDq()  << std::endl;
  std::cerr << prefix << "emergencyController.state:" << m_emergencyController.state  << std::endl;
  std::cerr << prefix << "emergencyController.dq:" << m_emergencyController.getMotorControllerDq() << std::endl;
  std::cerr << prefix << "tau:" << m_current_tau  << std::endl;
  std::cerr << prefix << "command_tauRef:" << m_command_tauRef  << std::endl;
  std::cerr << prefix << "actual_tauRef:" << m_actual_tauRef  << std::endl;
  std::cerr << std::endl;
}

// internal functions
void MotorTorqueController::setupControllerCommon(std::string _jname, double _dt)
{
  m_joint_name = _jname;
  m_dt = _dt;
  m_command_tauRef = 0.0;
  m_actual_tauRef = 0.0;
  m_normalController.state = INACTIVE;
  resetMotorControllerVariables(m_normalController);
  m_emergencyController.state = INACTIVE;
  resetMotorControllerVariables(m_emergencyController);
}

void MotorTorqueController::resetMotorControllerVariables(MotorTorqueController::MotorController& _mc)
{
  _mc.dq = 0;
  _mc.transition_dq = 0;
  _mc.recovery_dq = 0;
}

void MotorTorqueController::prepareStop(MotorTorqueController::MotorController &_mc)
{
  // angle difference to be recoverd
  _mc.transition_dq = _mc.getMotorControllerDq();

  // determine transition in 1 cycle
  _mc.recovery_dq = std::min(std::max(_mc.transition_dq / MAX_TRANSITION_COUNT, _mc.min_dq), _mc.max_dq); // transition in 1 cycle
  std::cerr << _mc.recovery_dq << std::endl;
  
  _mc.dq = 0; // dq must be reseted after recovery_dq setting(used in getMotoroControllerDq)
  _mc.state = STOP;
  return;
}

void MotorTorqueController::updateController(double _tau, double _tauRef, MotorTorqueController::MotorController& _mc)
{
  switch (_mc.state) {
  case ACTIVE:
    _mc.dq += _mc.controller->update(_tau, _tauRef);
    break;
  case STOP:
    if (std::abs(_mc.recovery_dq) > std::abs(_mc.transition_dq)){
        _mc.dq = 0;
        _mc.transition_dq = 0;
        _mc.state = INACTIVE;
        break;
      }
    _mc.transition_dq -= _mc.recovery_dq;
    break;
  default:
    _mc.controller->reset();
    resetMotorControllerVariables(_mc);
    break;
  }
  return;
}

// for MotorController
MotorTorqueController::MotorController::MotorController()
{
  state = INACTIVE;
  dq = 0;
  transition_dq = 0;
  recovery_dq = 0;
  setupTwoDofController(0.0, 0.0, 0.0);
}

MotorTorqueController::MotorController::~MotorController()
{
}

void MotorTorqueController::MotorController::setupTwoDofController(double _ke, double _tc, double _dt)
{
  controller.reset(new TwoDofController(_ke, _tc, _dt));
  controller->reset();
}

void MotorTorqueController::MotorController::setupTwoDofControllerPDModel(double _ke, double _kd, double _tc, double _dt)
{
  controller.reset(new TwoDofControllerPDModel(_ke, _kd, _tc, _dt));
  controller->reset();
}

void MotorTorqueController::MotorController::setupTwoDofControllerDynamicsModel(double _alpha, double _beta, double _ki, double _tc, double _dt)
{
  controller.reset(new TwoDofControllerDynamicsModel(_alpha, _beta, _ki, _tc, _dt));
  controller->reset();
}

double MotorTorqueController::MotorController::getMotorControllerDq(void)
{
  double ret_dq;
  switch(state) {
  case ACTIVE:
    ret_dq = dq + transition_dq; // if contorller interrupt its transition, base joint angle is not qRef, qRef + transition_dq
    break;
  case STOP:
    ret_dq = transition_dq;
    break;
  default:
    ret_dq = dq;
    break;
  }
  return ret_dq;
}
