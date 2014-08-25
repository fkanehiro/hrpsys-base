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

// interface class for TwoDofController
class TwoDofControllerInterface {
public:
  virtual ~TwoDofControllerInterface() {}
  virtual void reset() = 0; // initialze controller
  virtual void setup() = 0; // setup parameters
  virtual bool getParameter() = 0; // get prameter of controller
  virtual double update(double _x, double _xd) = 0; // calculate input from current value(_x) and target value(_xd)
};


class TwoDofController : public TwoDofControllerInterface {
public:
  struct TwoDofControllerParam {
    double ke; // gain
    double tc; // time constant
    double dt; // control cycle
  };
  TwoDofController(double _ke = 0, double _tc = 0, double _dt = 0, unsigned int _range = 0);
  ~TwoDofController();
  void setup();
  void setup(double _ke, double _tc, double _dt, unsigned int _range = 0);
  void reset();
  double update(double _x, double _xd);
  bool getParameter();
  bool getParameter(TwoDofControllerParam &_p);
private:
  TwoDofControllerParam param;
  Integrator integrator; // integrated (xd - x)
};

#endif // TWO_DOF_CONTROLLER_H
