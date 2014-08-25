// -*- C++ -*-
/*!
 * @file  TwoDofControllerDynamicsModel.h
 * @brief Feedback and Feedforward Controller which use PDModel as motor model
 * @date  $Date$
 *
 * $Id$
 */

#ifndef TWO_DOF_CONTROLLER_DYNAMICS_MODEL_H
#define TWO_DOF_CONTROLLER_DYNAMICS_MODEL_H

// </rtc-template>

#include "../Stabilizer/TwoDofController.h"
#include "Convolution.h"
#include <vector>

class TwoDofControllerDynamicsModel : public TwoDofControllerInterface {
public:
  struct TwoDofControllerDynamicsModelParam {
    double alpha; // completing squared param (s + alpha)^2 - beta^2
    double beta; // completing square param (s + alpha)^2 - beta^2
    double ki; // virtual inertia
    double tc; // time constant
    double dt; // control cycle
  };
  TwoDofControllerDynamicsModel(double _alpha = 0, double _beta = 0, double _ki = 0, double _tc = 0, double _dt = 0, unsigned int _range = 0);
  ~TwoDofControllerDynamicsModel();
  void setup();
  void setup(double _alpha, double _beta, double _ki, double _tc, double _dt, unsigned int _range = 0);
  void reset();
  double update(double _x, double _xd);
  bool getParameter();
  bool getParameter(TwoDofControllerDynamicsModelParam &_p);

private:
  TwoDofControllerDynamicsModelParam param;
  double current_time;
  Integrator integrate_exp_sinh_current;
  std::vector<double> exp_sinh;
  std::vector<Convolution> convolutions;
};

#endif // TWO_DOF_CONTROLLER_DYNAMICS_MODEL_H
