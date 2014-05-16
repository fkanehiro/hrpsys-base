// -*- C++ -*-

/*!
 * @file  TwoDofController.cpp
 * @brief Feedback and Feedforward Controller
 * @date  $Date$
 *
 * $Id$
 */

#include "Integrator.h"
#include <iostream>
#include <iterator>
#include <numeric>

Integrator::Integrator(double _dt, unsigned int _range) {
  setup(_dt, _range);
  reset();
}

Integrator::~Integrator(void) {
}

void Integrator::reset(void) {
  buffer.clear();
  return;
}

void Integrator::setup(double _dt, unsigned int _range) {
  dt = _dt;
  range = _range;
  return;
}

void Integrator::update (double _x) {
  // update buffer
  buffer.push_back(_x);
  if (buffer.size() > range) {
    buffer.pop_front(); // remove oldest data
  }
  return;
}

double Integrator::calculate(void) {
  // std::next/prev only use C++11 later
  std::deque<double>::iterator itr_first = buffer.begin();
  std::deque<double>::iterator itr_last = buffer.end();
  std::advance(itr_first, 1);
  std::advance(itr_first, -1);
  // calc integration by trapezoidal rule
  // sum(1/2 * (f(x_i) - f(x_(i+1))), 0, N) * dt = (1/2 * f(0) + sum(f(x_i), 1, N-1) + 1/2 * f(N)) * dt
  double integrate = (0.5 * buffer.front() + std::accumulate(itr_first, itr_last, 0.0) + 0.5 * buffer.back()) * dt;
  return integrate;
}
