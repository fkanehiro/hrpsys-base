// -*- C++ -*-
/*!
 * @file  Integrator.h
 * @brief Calcurate Integration
 * @date  $Date$
 *
 * $Id$
 */

#ifndef INTEGRATOR_H
#define INTEGRATOR_H

// </rtc-template>

#include <deque>

class Integrator {
public:
  // if range = 0, integrate from 0 to t. Otherwise, integrate from t - (range - 1) * dt to t.
  Integrator(double _dt = 0.005, unsigned int _range = 0);
  ~Integrator(void);
  void reset(void);
  void setup(double _dt, unsigned int _range);
  void update(double _x);
  double calculate(void);
private:
  double dt; // control cycle
  double first, sum, last; // for trapezoidal rule calculation
  bool init_integration_flag; // integration inited flag (true when first value is updated)
  unsigned int range; // integration range (from t_now - range * dt to t_now [sec])
  std::deque<double> buffer; // integration data buffer
};

#endif // TWO_DOF_CONTROLLER_H
