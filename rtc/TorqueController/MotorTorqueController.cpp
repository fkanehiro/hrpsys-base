// -*- C++ -*-

/*!
 * @file  MotorTorqueController.cpp
 * @brief torque controller for single motor
 * @date  
 *
 * $Id$
 */

#include "MotorTorqueController.h"
#include <iostream>
#include <cmath>

#define TRANSITION_TIME 2.0 // [sec]
#define MAX_TRANSITION_COUNT (TRANSITION_TIME/m_dt)
#define TORQUE_MARGIN 10.0 // [Nm]

MotorTorqueController::MotorTorqueController(std::string _jname, double _ke, double _tc, double _dt)
{
  m_joint_name = _jname;
  m_dt = _dt;
  m_command_tauRef = 0.0;
  m_actual_tauRef = 0.0;
  setupController(_ke, _tc, _dt);
  m_normalController.state = INACTIVE;
  resetMotorControllerVariables(m_normalController);
  m_emergencyController.state = INACTIVE;
  resetMotorControllerVariables(m_emergencyController);
}

MotorTorqueController::~MotorTorqueController(void)
{
}

void MotorTorqueController::setupController(double _ke, double _tc, double _dt)
{
  m_normalController.controller.setup(_ke, _tc, _dt);
  m_normalController.controller.reset();

  m_emergencyController.controller.setup(_ke, _tc, _dt);
  m_emergencyController.controller.reset();  
}

bool MotorTorqueController::activate(void)
{
  bool retval = false;
  if (m_normalController.state == INACTIVE) {
    resetMotorControllerVariables(m_normalController);
    m_normalController.controller.reset();
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
      m_emergencyController.controller.reset();
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
    limitedTauRef = copysign(_tauMax, _tau);
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
void MotorTorqueController::resetMotorControllerVariables(MotorTorqueController::MotorController& _mc)
{
  _mc.transition_count = 0;
  _mc.dq = 0;
  _mc.transition_dq = 0;
  _mc.recovery_dq = 0;
}

void MotorTorqueController::prepareStop(MotorTorqueController::MotorController &_mc)
{
  _mc.recovery_dq = _mc.getMotorControllerDq(); 
  _mc.transition_count = MAX_TRANSITION_COUNT;
  _mc.dq = 0; // dq must be reseted after recovery_dq setting(used in getMootroControllerDq)
  _mc.state = STOP;
  return;
}

void MotorTorqueController::updateController(double _tau, double _tauRef, MotorTorqueController::MotorController& _mc)
{
  switch (_mc.state) {
  case ACTIVE:
    _mc.dq += _mc.controller.update(_tau, _tauRef);
    break;
  case STOP:
    if (_mc.transition_count < 0){
        _mc.dq = 0;
        _mc.transition_dq = 0;
        _mc.state = INACTIVE;
        break;
      }
    _mc.transition_dq = (_mc.recovery_dq / MAX_TRANSITION_COUNT) * _mc.transition_count;
    _mc.transition_count--;
    break;
  default:
    _mc.controller.reset();
    resetMotorControllerVariables(_mc);
    break;
  }
  return;
}

// for MotorController
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
