// -*- C++ -*-

/*!
 * @file  TwoDofControllerPDModel.cpp
 * @brief Feedback and Feedforward Controller which use PDModel as motor model
 * @date  $Date$
 *
 * $Id$
 */

#include "TwoDofControllerPDModel.h"
#include <iostream>
#include <cmath>

#define NUM_CONVOLUTION_TERM 3

TwoDofControllerPDModel::TwoDofControllerPDModel() {
  param = TwoDofControllerPDModel::TwoDofControllerPDModelParam(); // use default constructor
  current_time = 0;
  convolutions.clear();
  for (int i = 0; i < NUM_CONVOLUTION_TERM; i++) {
    convolutions.push_back(Convolution(0.0, 0.0));
  }
  error_prefix = ""; // inheritted from TwoDofControllerInterface  
}

TwoDofControllerPDModel::TwoDofControllerPDModel(TwoDofControllerPDModel::TwoDofControllerPDModelParam &_param, unsigned int _range) {
  param.ke = _param.ke; param.kd = _param.kd; param.tc = _param.tc; param.dt = _param.dt;
  current_time = 0;
  convolutions.clear();
  for (int i = 0; i < NUM_CONVOLUTION_TERM; i++) {
    convolutions.push_back(Convolution(_param.dt, _range));
  }
  error_prefix = ""; // inheritted from TwoDofControllerInterface  
}

TwoDofControllerPDModel::~TwoDofControllerPDModel() {
}

void TwoDofControllerPDModel::setup() {
  param.ke = 0; param.kd = 0; param.tc = 0; param.dt = 0;
  convolutions.clear();
  reset();
}

void TwoDofControllerPDModel::setup(TwoDofControllerPDModel::TwoDofControllerPDModelParam &_param, unsigned int _range) {
  param.ke = _param.ke; param.kd = _param.kd; param.tc = _param.tc; param.dt = _param.dt;
  convolutions.clear();
  for (int i = 0; i < NUM_CONVOLUTION_TERM; i++) {
    convolutions.push_back(Convolution(_param.dt, _range));
  }
  reset();
}

bool TwoDofControllerPDModel::getParameter() {
  return false;
}

bool TwoDofControllerPDModel::getParameter(TwoDofControllerPDModel::TwoDofControllerPDModelParam &_p) {
  _p.ke = param.ke;
  _p.kd = param.kd;
  _p.tc = param.tc;
  _p.dt = param.dt;
  return true;
}

void TwoDofControllerPDModel::reset() {
  current_time = 0;
  for (std::vector<Convolution>::iterator itr = convolutions.begin(); itr != convolutions.end(); ++itr) {
    (*itr).reset();
  }
}

double TwoDofControllerPDModel::update (double _x, double _xd) {
  // motor model: P = -ke / s + kd
  
  double velocity; // velocity calcurated by 2 dof controller

  // check parameters
  if (!param.ke || !param.kd || !param.tc || !param.dt) {
    std::cerr << "[" << error_prefix << "]" << "TwoDofControllerPDModel parameters are not set." << std::endl;
    std::cerr << "[" << error_prefix << "]" << "ke: " << param.ke << ", kd: " << param.kd << ", tc: " << param.tc << ", dt: " << param.dt << std::endl;
    return 0;
  }

  // update convolution
  convolutions[0].update(std::exp((param.ke / param.kd) * current_time), _x);
  convolutions[1].update(std::exp((param.ke / param.kd) * current_time), _xd - _x);
  convolutions[2].update(1 - std::exp((param.ke / param.kd) * current_time), _xd - _x);

  // 2 dof controller
  velocity = (1 / (param.tc * param.kd)) * (-convolutions[0].calculate() + convolutions[1].calculate())
    - (1 / (param.tc * param.tc * param.ke)) * convolutions[2].calculate();

  current_time += param.dt;
  
  return velocity * param.dt;
  
}
