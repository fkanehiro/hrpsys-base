// -*- C++ -*-
/*!
 * @file  Convolution.h
 * @brief Convolution Calculator
 * @date  $Date$
 *
 * $Id$
 */

#ifndef CONVOLUTION_H
#define CONVOLUTION_H

// </rtc-template>

#include "../Stabilizer/Integrator.h"

class Convolution {
public:
  // if range = 0, integrate from 0 to t. Otherwise, integrate from t - (range - 1) * dt to t.
  Convolution(double _dt = 0.005, unsigned int _range = 0);
  ~Convolution(void);
  void reset(void);
  void setup(double _dt, unsigned int _range);
  void update(double _f, double _g);
  double calculate(void);
private:
  double dt; // control cycle
  unsigned int range; // integration range (from t_now - range * dt to t_now [sec])
  std::deque<double> f_buffer; // integration data buffer for f
  std::deque<double> g_buffer; // integration data buffer for g
  long long buffer_size; // buffer size of convolution values (f, g)
  Integrator integrator; // convolution(f, g) = integrate(f(x)*g(t-x), x=0, x=t);
};

#endif // CONVOLUTION_H
