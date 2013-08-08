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

class TwoDofController {
public:
  TwoDofController(double _ke =0, double _tc=0, double _dt=0);
  ~TwoDofController();
  void reset();
  void setup(double _ke, double _tc, double _dt);
  double update(double _x, double _xd);
private:
  double ke, tc, dt; // ke: gain, tc: time constant, dt: control cycle
  double pdq;
};

#endif // TWO_DOF_CONTROLLER_H
