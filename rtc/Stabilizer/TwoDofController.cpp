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
  pdq = 0;
}

void TwoDofController::setup(double _ke, double _tc, double _dt) {
  ke = _ke;
  tc = _tc;
  dt = _dt;
  pdq = 0;
}

double TwoDofController::update (double _x, double _xd) {
  // Ca = 1/P * Q / (1-Q)
  // Cb = Gr / (1-Gr) * 1/P * 1 / (1-Q)
  // P = - ke/s
  // Gr = Q = 1 / (tc*s + 1)

  // integrate (xd - x)
  //integrated_diff += (_xd - _x) * dt;

  // 2 dof controller
  double Q = 1/(tc*dt+1);
  double Gr = 1/(tc*dt+1);
  double P = (-ke/dt);
  double Ca = 1/P * Q/(1-Q);
  double Cb = Gr/ (1-Gr) * 1/P * 1/(1-Q);

  double dv = -1 * Ca * _x + Cb * (_xd - _x);
  //  std::cerr << "TDC " << dv << " " << Ca << " " << Cb << std::endl;
  pdq += dv * dt;

  return pdq;
  
}
