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
  class TwoDofControllerPDModelParam {
  public:
    TwoDofControllerPDModelParam() {
      ke = kd = tc = dt = 0.0; // set default param
    }
    ~TwoDofControllerPDModelParam() {
    }
    static int getControllerParamNum() {
      return 3;
    }
    double ke; // Pgain
    double kd; // Dgain
    double tc; // time constant
    double dt; // control cycle (not controller but system parameter)
  };
  TwoDofControllerPDModel();
  TwoDofControllerPDModel(TwoDofControllerPDModelParam &_param, unsigned int _range = 0);
  ~TwoDofControllerPDModel();
  void setup();
  void setup(TwoDofControllerPDModelParam &_param, unsigned int _range = 0);
  void reset();
  double update(double _x, double _xd);
  bool getParameter();
  bool getParameter(TwoDofControllerPDModelParam &_p);
private:
  TwoDofControllerPDModelParam param;
  double current_time;
  std::vector<Convolution> convolutions;
};

#endif // TWO_DOF_CONTROLLER_PDMODEL_H
