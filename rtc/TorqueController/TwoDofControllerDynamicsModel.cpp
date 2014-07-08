// -*- C++ -*-

/*!
 * @file  TwoDofControllerDynamicsModel.cpp
 * @brief Feedback and Feedforward Controller which use PDModel as motor model
 * @date  $Date$
 *
 * $Id$
 */

#include "TwoDofControllerDynamicsModel.h"
#include <iostream>
#include <cmath>

#define NUM_CONVOLUTION_TERM 3

TwoDofControllerDynamicsModel::TwoDofControllerDynamicsModel(double _alpha, double _beta, double _ki, double _tc, double _dt, unsigned int _range) {
  alpha = _alpha; beta = _beta; ki = _ki; tc = _tc; dt = _dt;
  current_time = 0;
  convolutions.clear();
  exp_sinh.clear();
  for (int i = 0; i < NUM_CONVOLUTION_TERM; i++) {
    convolutions.push_back(Convolution(_dt, _range));
  }
  integrate_exp_sinh_current.setup(_dt, _range);
}

TwoDofControllerDynamicsModel::~TwoDofControllerDynamicsModel() {
}

void TwoDofControllerDynamicsModel::setup() {
  alpha = 0; beta = 0; ki = 0; tc = 0; dt = 0;
  convolutions.clear();
  exp_sinh.clear();
  integrate_exp_sinh_current.reset();
}

void TwoDofControllerDynamicsModel::setup(double _alpha, double _beta, double _ki, double _tc, double _dt, unsigned int _range) {
  alpha = _alpha; beta = _beta; ki = _ki; tc = _tc; dt = _dt;
  convolutions.clear();
  for (int i = 0; i < NUM_CONVOLUTION_TERM; i++) {
    convolutions.push_back(Convolution(_dt, _range));
  }
  integrate_exp_sinh_current.setup(_dt, _range);
  reset();
}

void TwoDofControllerDynamicsModel::reset() {
  current_time = 0;
  exp_sinh.clear();
  for (std::vector<Convolution>::iterator itr = convolutions.begin(); itr != convolutions.end(); ++itr) {
    (*itr).reset();
  }
  integrate_exp_sinh_current.reset();
}

double TwoDofControllerDynamicsModel::update (double _x, double _xd) {
  // motor model: P = -ke / s + kd + ki * s
  // completing the square: s^2 + (kd/ki)*s - (ke/ki) = (s+alpha)^2-beta^2
  
  double velocity; // velocity calcurated by 2 dof controller

  // check parameters
  if (!alpha || !beta || !tc || !dt) {
    std::cerr << "ERROR: parameters are not set." << std::endl;
    std::cerr << "alpha: " << alpha << ", beta: " << beta << ", tc: " << tc << ", dt: " << dt << std::endl;
    return 0;
  }
  
  // update exp(-a*t)*sinh(b*t) buffer
  double exp_sinh_current = std::exp(-alpha * current_time) * std::sinh(beta * current_time);
  exp_sinh.push_back(exp_sinh_current);
  integrate_exp_sinh_current.update(exp_sinh_current);

  // update convolution
  convolutions[0].update(exp_sinh_current, _x);
  convolutions[1].update(exp_sinh_current, _xd - _x);
  convolutions[2].update(integrate_exp_sinh_current.calculate(), _xd - _x);

  // 2 dof controller
  velocity = (1 / (tc * ki * beta)) * (-convolutions[0].calculate() + convolutions[1].calculate())
    + (1 / (tc * tc * ki * beta)) * convolutions[2].calculate();

  current_time += dt;
  
  return velocity * dt;
  
}
