// -*- C++ -*-
/*!
 * @file  MotorTorqueController.h
 * @brief torque controller for single motor
 * @date  $Date$
 *
 * $Id$
 */

#ifndef MOTOR_TORQUE_CONTROLLER_H
#define MOTOR_TORQUE_CONTROLLER_H

#include <string>
#include "../Stabilizer/TwoDofController.h"

// </rtc-template>

class MotorTorqueController {
public:

  enum controller_state_t {
    INACTIVE, // dq = 0
    STOP, // resume
    ACTIVE // execute torque control
  }; 
  MotorTorqueController(std::string _jname = "", double ke = 0, double tc = 0, double dt = 0);
  ~MotorTorqueController(void);

  void setupController(double _ke, double _tc, double _dt);
  bool activate(void); // set state of torque controller to ACTIVE
  bool deactivate(void); // set state of torque controller to STOP -> INACTIVE
  bool setReferenceTorque(double _tauRef); // set reference torque (does not activate controller)
  double execute(double _tau, double _tauMax); // determine final state and tauRef, then throw tau, tauRef and state to executeControl
  
  // accessor
  std::string getJointName(void);
  controller_state_t getMotorControllerState(void);

  // for debug
  void printMotorControllerVariables(void); // debug print
  
private:

  class MotorController {
  public:
    TwoDofController controller;
    controller_state_t state;
    double transition_count;
    double dq; //difference of joint angle from base(qRef) from tdc. it is calcurated by dq = integrate(qd * dt), dq*dt is output of tdc 
    double transition_dq; // for transition
    double recovery_dq; // last difference of joint angle from qRef (dq + transition_dq) when state was changed to STOP
    double getMotorControllerDq(void); // get according dq according to state
  };
  
  // internal functions
  void resetMotorControllerVariables(MotorController& _mc); // reset internal torque control parameter  
  void prepareStop(MotorController &_mc);
  void updateController(double _tau, double _tauRef, MotorController& _mc); // execute control and update controller member valiables 

  std::string m_joint_name; // joint name which is controled
  int m_transition_count; // positive value when stopping
  double m_dt; // control term
  double m_current_tau; // current tau (mainly for debug message)
  double m_command_tauRef; // reference tau
  double m_actual_tauRef; // reference tau which is limited or overwritten by emergency (mainly for debug message)
  MotorController m_normalController; // substance of two dof controller
  MotorController m_emergencyController; // overwrite normal controller when emergency
};


#endif // MOTOR_TORQUE_CONTROLLER_H
