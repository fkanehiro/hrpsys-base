// -*- C++ -*-

/*!
 * @file  Convolution.cpp
 * @brief Calculate Convolution
 * @date  $Date$
 *
 * $Id$
 */

#include "Convolution.h"
#include <vector>

Convolution::Convolution(double _dt, unsigned int _range) {
  integrator = Integrator();
  setup(_dt, _range);
}

Convolution::~Convolution(void) {
}

void Convolution::reset(void) {
  f_buffer.clear();
  g_buffer.clear();
  integrator.reset();
  buffer_size = 0;
  return;
}

void Convolution::setup(double _dt, unsigned int _range) {
  dt = _dt;
  range = _range;
  integrator.setup(_dt, 0); // integrator range is inf (if range of integrator > 0, integrator remove f(0)*g(t) first)
  reset();
  return;
}

void Convolution::update (double _f, double _g) {
  f_buffer.push_back(_f);
  g_buffer.push_back(_g);
  buffer_size++;
  if (range > 0 && buffer_size > range) { // restrict buffer size
    f_buffer.pop_front();
    g_buffer.pop_front();
    buffer_size--;
  }
  return;
}

double Convolution::calculate(void) {
  // calc f(x) * g(t-x) buffer
  std::vector<double> fg_buffer;
  for (int i = 0; i < buffer_size; i++) {
    fg_buffer.push_back(f_buffer[i] * g_buffer[(buffer_size -1) - i]);
  }
  // calc integration
  integrator.reset();
  for (int i = 0; i < buffer_size; i++) {
    integrator.update(fg_buffer[i]);
  }
  return integrator.calculate();
}
