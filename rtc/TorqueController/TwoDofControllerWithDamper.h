// -*- C++ -*-
/*!
 * @file  TwoDofControllerWithDamper.h
 * @brief Feedback and Feedforward Controller which use PDModel as motor model
 * @date  $Date$
 *
 * $Id$
 */

#ifndef TWO_DOF_CONTROLLER_WITH_DAMPER_H
#define TWO_DOF_CONTROLLER_WITH_DAMPER_H

// </rtc-template>

#include "../Stabilizer/TwoDofController.h"
#include "Convolution.h"
#include <vector>

class TwoDofControllerWithDamper : public TwoDofController {
public:
  TwoDofControllerWithDamper(double _ke = 0, double _kd = 0, double _tc = 0, double _dt = 0, unsigned int _range = 0);
  ~TwoDofControllerWithDamper();
  void reset();
  void setup(double _ke, double _kd, double _tc, double _dt, unsigned int _range = 0);
  double update(double _x, double _xd);
private:
  double ke, kd, tc, dt; // ke: Pgain, kd: Dgain, tc: time constant, dt: control cycle
  double current_time;
  std::vector<Convolution> convolutions;
};

#endif // TWO_DOF_CONTROLLER_WITH_DAMPER_H
