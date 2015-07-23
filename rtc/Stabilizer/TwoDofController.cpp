// -*- C++ -*-

/*!
 * @file  TwoDofController.cpp
 * @brief Feedback and Feedforward Controller
 * @date  $Date$
 *
 * $Id$
 */

#include "TwoDofController.h"
#include <iostream>

void TwoDofControllerInterface::setErrorPrefix(const std::string& _error_prefix) {
  error_prefix = _error_prefix;
}

TwoDofController::TwoDofController() {
  param = TwoDofController::TwoDofControllerParam(); // use default constructor
  integrator = Integrator(0.0, 0.0);
  integrator.reset();
  error_prefix = "";
}

TwoDofController::TwoDofController(TwoDofController::TwoDofControllerParam &_param, unsigned int _range) {
  param.ke = _param.ke; param.tc = _param.tc; param.dt = _param.dt;
  integrator = Integrator(_param.dt, _range);
  integrator.reset();
  error_prefix = "";
}

TwoDofController::~TwoDofController() {
}

void TwoDofController::setup() {
  param.ke = 0; param.tc = 0; param.dt = 0;
  integrator = Integrator(0, 0);
  reset();
}

void TwoDofController::setup(TwoDofController::TwoDofControllerParam &_param, unsigned int _range) {
  param.ke = _param.ke; param.tc = _param.tc; param.dt = _param.dt;
  integrator = Integrator(_param.dt, _range);
  reset();
}

void TwoDofController::reset() {
  integrator.reset();
}

bool TwoDofController::getParameter() {
  return false;
}

bool TwoDofController::getParameter(TwoDofController::TwoDofControllerParam &_p) {
  _p.ke = param.ke;
  _p.tc = param.tc;
  _p.dt = param.dt;
  return true;
}

double TwoDofController::update (double _x, double _xd) {
  // Ca = 1/P * Q / (1-Q)
  // Cb = Gr / (1-Gr) * 1/P * 1 / (1-Q)
  // P = - ke/s
  // Gr = Q = 1 / (tc*s + 1)

  double velocity; // velocity calcurated by 2 dof controller

  // check parameters
  if (!param.ke || !param.tc || !param.dt){
    std::cerr << "[" << error_prefix << "]" << "TwoDofController parameters are not set." << std::endl;
    return 0;
  }
  
  // integrate (xd - x)
  // integrated_diff += (_xd - _x) * dt;
  integrator.update(_xd - _x);

  // 2 dof controller
  velocity = (-_x + (_xd - _x) + (integrator.calculate() / param.tc)) / (-param.ke * param.tc);

  return -velocity * param.dt;
  
}

// for compatiblity of Stabilizer 
TwoDofController::TwoDofController(double _ke, double _tc, double _dt, unsigned int _range) {
  param.ke = _ke; param.tc = _tc; param.dt = _dt;
  integrator = Integrator(_dt, _range);
  integrator.reset();
  error_prefix = "";
}

void TwoDofController::setup(double _ke, double _tc, double _dt, unsigned int _range) {
  param.ke = _ke; param.tc = _tc; param.dt = _dt;
  integrator = Integrator(_dt, _range);
}
