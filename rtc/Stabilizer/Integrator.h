// -*- C++ -*-
/*!
 * @file  Integrator.h
 * @brief Feedback and Feedforward Controller
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
  Integrator(double _dt = 0.005, unsigned int _range = 1000);
  ~Integrator(void);
  void reset(void);
  void setup(double _dt, unsigned int _range);
  void update(double _x);
  double calculate(void);
private:
  double dt; // control cycle
  unsigned int range; // integration range (from t_now - range * dt to t_now [sec])
  std::deque<double> buffer; // integration data buffer
};

#endif // TWO_DOF_CONTROLLER_H
