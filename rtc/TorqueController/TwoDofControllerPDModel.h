// -*- C++ -*-
/*!
 * @file  TwoDofControllerPDModel.h
 * @brief Feedback and Feedforward Controller which use PDModel as motor model
 * @date  $Date$
 *
 * $Id$
 */

#ifndef TWO_DOF_CONTROLLER_PDMODEL_H
#define TWO_DOF_CONTROLLER_PDMODEL_H

// </rtc-template>

#include "../Stabilizer/TwoDofController.h"
#include "Convolution.h"
#include <vector>

class TwoDofControllerPDModel : public TwoDofControllerInterface {
public:
  TwoDofControllerPDModel(double _ke = 0, double _kd = 0, double _tc = 0, double _dt = 0, unsigned int _range = 0);
  ~TwoDofControllerPDModel();
  void setup();
  void setup(double _ke, double _kd, double _tc, double _dt, unsigned int _range = 0);
  void reset();
  double update(double _x, double _xd);
private:
  double ke, kd, tc, dt; // ke: Pgain, kd: Dgain, tc: time constant, dt: control cycle
  double current_time;
  std::vector<Convolution> convolutions;
};

#endif // TWO_DOF_CONTROLLER_PDMODEL_H
