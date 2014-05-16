// -*- C++ -*-
/*!
 * @file  TwoDofController.h
 * @brief Feedback and Feedforward Controller
 * @date  $Date$
 *
 * $Id$
 */

#ifndef TWO_DOF_CONTROLLER_H
#define TWO_DOF_CONTROLLER_H

// </rtc-template>

#include "Integrator.h"

class TwoDofController {
public:
  TwoDofController(double _ke = 0, double _tc = 0, double _dt = 0, unsigned int _range = 0);
  ~TwoDofController();
  void reset();
  void setup(double _ke, double _tc, double _dt, unsigned int _range = 0);
  double update(double _x, double _xd);
private:
  double ke, tc, dt; // ke: gain, tc: time constant, dt: control cycle
  Integrator integrator; // integrated (xd - x)
};

#endif // TWO_DOF_CONTROLLER_H
