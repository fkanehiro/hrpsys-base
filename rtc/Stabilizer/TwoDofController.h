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
#include <string>

// interface class for TwoDofController
class TwoDofControllerInterface {
public:
  virtual ~TwoDofControllerInterface() {}
  virtual void reset() = 0; // initialze controller
  virtual void setup() = 0; // setup parameters
  virtual bool getParameter() = 0; // get prameter of controller
  virtual double update(double _x, double _xd) = 0; // calculate input from current value(_x) and target value(_xd)
  void setErrorPrefix(const std::string& _error_prefix); // set prefix string for error message
protected:
  std::string error_prefix;
};


class TwoDofController : public TwoDofControllerInterface {
public:
  class TwoDofControllerParam {
  public:
    TwoDofControllerParam() {
      ke = tc = dt = 0.0; // set default param
    }
    ~TwoDofControllerParam() {
    }
    static int getControllerParamNum() {
      return 2;
    }
    double ke; // gain
    double tc; // time constant
    double dt; // control cycle (not controller but system parameter)
  };
  TwoDofController();
  TwoDofController(TwoDofControllerParam &_param, unsigned int _range = 0);
  ~TwoDofController();
  void setup();
  void setup(TwoDofControllerParam &_param, unsigned int _range = 0);
  void reset();
  double update(double _x, double _xd);
  bool getParameter();
  bool getParameter(TwoDofControllerParam &_p);

  // for compatibility of Stabilizer. TODO: replace to new parameter argument
  TwoDofController(double _ke, double _tc, double _dt, unsigned int _range = 0);
  void setup(double _ke, double _tc, double _dt, unsigned int _range = 0);
  
private:
  TwoDofControllerParam param;
  Integrator integrator; // integrated (xd - x)
};

#endif // TWO_DOF_CONTROLLER_H
