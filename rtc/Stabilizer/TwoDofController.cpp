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

TwoDofController::TwoDofController(double _ke, double _tc, double _dt) {
  setup(_ke, _tc, _dt);
  reset();
}

TwoDofController::~TwoDofController() {
}

void TwoDofController::reset() {
  integrated_diff = 0;
}

void TwoDofController::setup(double _ke, double _tc, double _dt) {
  ke = _ke; tc = _tc; dt = _dt;
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
  integrated_diff += (_xd - _x) * dt;

  // 2 dof controller
  velocity = (-_x + (_xd - _x) + (integrated_diff / tc)) / (-ke * tc);

  return velocity * dt;
  
}
