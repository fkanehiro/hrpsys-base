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

TwoDofController::TwoDofController(double _ke, double _tc, double _dt, unsigned int _range) {
  ke = _ke; tc = _tc; dt = _dt;
  integrator = Integrator(_dt, _range);
  integrator.reset();
}

TwoDofController::~TwoDofController() {
}

void TwoDofController::setup() {
  ke = 0; tc = 0; dt = 0;
  integrator = Integrator(0, 0);
}

void TwoDofController::setup(double _ke, double _tc, double _dt, unsigned int _range) {
  ke = _ke; tc = _tc; dt = _dt;
  integrator = Integrator(_dt, _range);
}

void TwoDofController::reset() {
  integrator.reset();
}

double TwoDofController::update (double _x, double _xd) {
  // Ca = 1/P * Q / (1-Q)
  // Cb = Gr / (1-Gr) * 1/P * 1 / (1-Q)
  // P = - ke/s
  // Gr = Q = 1 / (tc*s + 1)

  double velocity; // velocity calcurated by 2 dof controller

  // check parameters
  if (!ke || !tc || !dt){
    std::cerr << "ERROR: parameters are not set." << std::endl;
    return 0;
  }
  
  // integrate (xd - x)
  // integrated_diff += (_xd - _x) * dt;
  integrator.update(_xd - _x);

  // 2 dof controller
  velocity = (-_x + (_xd - _x) + (integrator.calculate() / tc)) / (-ke * tc);

  return -velocity * dt;
  
}
