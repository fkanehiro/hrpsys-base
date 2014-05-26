// -*- C++ -*-

/*!
 * @file  Integrator.cpp
 * @brief Calcurate Integration
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
}

Integrator::~Integrator(void) {
}

void Integrator::reset(void) {
  buffer.clear();
  first = 0;
  sum = 0;
  last = 0;
  init_integration_flag = false;
  return;
}

void Integrator::setup(double _dt, unsigned int _range) {
  dt = _dt;
  range = _range;
  reset();
  return;
}

void Integrator::update (double _x) {

  // integration by trapezoidal rule:
  // (1/2 * first + sum(f(x_i), 1, N-1) + 1/2 * last) * dt
  if (!init_integration_flag) {
    first = _x; // update first value
    init_integration_flag = true;
    // first value is not counted to sum
  } else {
    sum += last; // sum is last is assumed to be 0 at first
    last = _x;
  }

  // if integration range is defined, use buffer
  if (range > 0) {
    buffer.push_back(_x); // save values (include first)
    if (buffer.size() > range) {
      buffer.pop_front(); // remove oldest data
      first = buffer.front(); // update first value
      sum -= first;
    }
  }
  
  return;
}

double Integrator::calculate(void) {
  // calc integration by trapezoidal rule
  // sum(1/2 * (f(x_i) - f(x_(i+1))), 0, N) * dt = (1/2 * f(0) + sum(f(x_i), 1, N-1) + 1/2 * f(N)) * dt
  return (0.5 * first + sum + 0.5 * last) * dt;
}
