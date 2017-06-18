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

TwoDofControllerDynamicsModel::TwoDofControllerDynamicsModel() {
  param = TwoDofControllerDynamicsModel::
      TwoDofControllerDynamicsModelParam();  // use default constructor
  current_time = 0;
  convolutions.clear();
  exp_sinh.clear();
  for (int i = 0; i < NUM_CONVOLUTION_TERM; i++) {
    convolutions.push_back(Convolution(0.0, 0.0));
  }
  integrate_exp_sinh_current.setup(0.0, 0.0);
  error_prefix = "";  // inheritted from TwoDofControllerInterface
}

TwoDofControllerDynamicsModel::TwoDofControllerDynamicsModel(
    TwoDofControllerDynamicsModel::TwoDofControllerDynamicsModelParam &_param,
    unsigned int _range) {
  param.alpha = _param.alpha;
  param.beta = _param.beta;
  param.ki = _param.ki;
  param.tc = _param.tc;
  param.dt = _param.dt;
  current_time = 0;
  convolutions.clear();
  exp_sinh.clear();
  for (int i = 0; i < NUM_CONVOLUTION_TERM; i++) {
    convolutions.push_back(Convolution(_param.dt, _range));
  }
  integrate_exp_sinh_current.setup(_param.dt, _range);
  error_prefix = "";  // inheritted from TwoDofControllerInterface
}

TwoDofControllerDynamicsModel::~TwoDofControllerDynamicsModel() {}

void TwoDofControllerDynamicsModel::setup() {
  param.alpha = 0;
  param.beta = 0;
  param.ki = 0;
  param.tc = 0;
  param.dt = 0;
  convolutions.clear();
  exp_sinh.clear();
  integrate_exp_sinh_current.reset();
  reset();
}

void TwoDofControllerDynamicsModel::setup(
    TwoDofControllerDynamicsModel::TwoDofControllerDynamicsModelParam &_param,
    unsigned int _range) {
  param.alpha = _param.alpha;
  param.beta = _param.beta;
  param.ki = _param.ki;
  param.tc = _param.tc;
  param.dt = _param.dt;
  convolutions.clear();
  for (int i = 0; i < NUM_CONVOLUTION_TERM; i++) {
    convolutions.push_back(Convolution(_param.dt, _range));
  }
  integrate_exp_sinh_current.setup(_param.dt, _range);
  reset();
}

void TwoDofControllerDynamicsModel::reset() {
  current_time = 0;
  exp_sinh.clear();
  for (std::vector<Convolution>::iterator itr = convolutions.begin();
       itr != convolutions.end(); ++itr) {
    (*itr).reset();
  }
  integrate_exp_sinh_current.reset();
}

bool TwoDofControllerDynamicsModel::getParameter() { return false; }

bool TwoDofControllerDynamicsModel::getParameter(
    TwoDofControllerDynamicsModelParam &_p) {
  _p.alpha = param.alpha;
  _p.beta = param.beta;
  _p.ki = param.ki;
  _p.tc = param.tc;
  _p.dt = param.dt;
  return true;
}

double TwoDofControllerDynamicsModel::update(double _x, double _xd) {
  // motor model: P = -ke / s + kd + ki * s
  // completing the square: s^2 + (kd/ki)*s - (ke/ki) = (s+alpha)^2-beta^2

  double velocity;  // velocity calcurated by 2 dof controller

  // check parameters
  if (!param.alpha || !param.beta || !param.tc || !param.dt) {
    std::cerr << "[" << error_prefix << "]"
              << "TwoDofControllerDynamicsModel parameters are not set."
              << std::endl;
    std::cerr << "[" << error_prefix << "]"
              << "alpha: " << param.alpha << ", beta: " << param.beta
              << ", tc: " << param.tc << ", dt: " << param.dt << std::endl;
    return 0;
  }

  // update exp(-a*t)*sinh(b*t) buffer
  double exp_sinh_current = std::exp(-param.alpha * current_time) *
                            std::sinh(param.beta * current_time);
  exp_sinh.push_back(exp_sinh_current);
  integrate_exp_sinh_current.update(exp_sinh_current);

  // update convolution
  convolutions[0].update(exp_sinh_current, _x);
  convolutions[1].update(exp_sinh_current, _xd - _x);
  convolutions[2].update(integrate_exp_sinh_current.calculate(), _xd - _x);

  // 2 dof controller
  velocity = (1 / (param.tc * param.ki * param.beta)) *
                 (-convolutions[0].calculate() + convolutions[1].calculate()) +
             (1 / (param.tc * param.tc * param.ki * param.beta)) *
                 convolutions[2].calculate();

  current_time += param.dt;

  return velocity * param.dt;
}
