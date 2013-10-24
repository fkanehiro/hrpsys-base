// -*- C++ -*-
/*!
 * @file  MotorHeatParam.h
 * @brief motor heat parmaeter structure
 * @date  $Date$
 *
 * $Id$
 */

#ifndef MOTOR_HEAT_PARAM_H
#define MOTOR_HEAT_PARAM_H


// </rtc-template>
class MotorHeatParam
{
 public:
  // Tnew = T + (P - ((T - Ta) / R) * dt) / C
  //      = T + ((Re*K^2/C) * tau^2) + ((1/RC) * (T - Ta) * dt)
  // * P = Re * I^2 = Re * (K * tau)^2
  
  double temperature; // current temperature
  double currentCoeffs; // Re*K^2/C
  double thermoCoeffs; // 1/RC

  MotorHeatParam(){
    defaultParams();
  }
  
  // default params for motor heat param
  void defaultParams(){
    temperature = 30.0;
    currentCoeffs = 0.00003;
    thermoCoeffs = 0.001;
  } 

};

#endif // MOTOR_HEAT_PARAM_H
