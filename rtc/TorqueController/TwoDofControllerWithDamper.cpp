// -*- C++ -*-

/*!
 * @file  TwoDofControllerWithDamer.cpp
 * @brief Feedback and Feedforward Controller which use PDModel as motor model
 * @date  $Date$
 *
 * $Id$
 */

#include "TwoDofControllerWithDamper.h"
#include <iostream>
#include <cmath>

#define NUM_CONVOLUTION_TERM 3

TwoDofControllerWithDamper::TwoDofControllerWithDamper(double _ke, double _kd, double _tc, double _dt, unsigned int _range) {
  setup(_ke, _kd, _tc, _dt, _range);
  reset();
}

TwoDofControllerWithDamper::~TwoDofControllerWithDamper() {
}

void TwoDofControllerWithDamper::reset() {
  current_time = 0;
  for (std::vector<Convolution>::iterator itr = convolutions.begin(); itr != convolutions.end(); ++itr) {
    (*itr).reset();
  }
}

void TwoDofControllerWithDamper::setup(double _ke, double _kd, double _tc, double _dt, unsigned int _range) {
  ke = _ke; kd = _kd; tc = _tc; dt = _dt;
  convolutions.clear();
  for (int i = 0; i < NUM_CONVOLUTION_TERM; i++) {
    convolutions.push_back(Convolution(_dt, _range));
  }
  reset();
}

double TwoDofControllerWithDamper::update (double _x, double _xd) {
  // motor model: P = -ke / s + kd
  
  double velocity; // velocity calcurated by 2 dof controller

  // check parameters
  if (!ke || !kd || !tc || !dt) {
    std::cerr << "ERROR: parameters are not set." << std::endl;
    std::cerr << "ke: " << ke << ", kd: " << kd << ", tc: " << tc << ", dt: " << dt << std::endl;
    return 0;
  }

  // update convolution
  convolutions[0].update(std::exp((ke / kd) * current_time), _x);
  convolutions[1].update(std::exp((ke / kd) * current_time), _xd - _x);
  convolutions[2].update(1 - std::exp((ke / kd) * current_time), _xd - _x);

  // 2 dof controller
  velocity = (1 / (tc * kd)) * (-convolutions[0].calculate() + convolutions[1].calculate())
    - (1 / (tc * tc * ke)) * convolutions[2].calculate();

  current_time += dt;
  
  return velocity * dt;
  
}
