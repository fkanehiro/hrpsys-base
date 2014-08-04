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
#include <boost/shared_ptr.hpp>
#include "../Stabilizer/TwoDofController.h"
#include "TwoDofControllerPDModel.h"
#include "TwoDofControllerDynamicsModel.h"

// </rtc-template>

class MotorTorqueController {
public:

  enum controller_state_t {
    INACTIVE, // dq = 0
    STOP, // resume
    ACTIVE // execute torque control
  };
  MotorTorqueController();
  MotorTorqueController(std::string _jname, double ke, double tc, double dt);
  MotorTorqueController(std::string _jname, double ke, double kd, double tc, double dt);
  MotorTorqueController(std::string _jname, double _alpha, double _beta, double _ki, double _tc, double _dt);
  ~MotorTorqueController(void);

  void setupController(double _ke, double _tc, double _dt);
  void setupController(double _ke, double _kd, double _tc, double _dt);
  void setupController(double _alpha, double _beta, double _ki, double _tc, double _dt);
  void setupMotorControllerMinMaxDq(double _min_dq, double _max_dq); // set min/max dq for transition
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
    MotorController();
    ~MotorController();
    boost::shared_ptr<TwoDofControllerInterface> controller;
    controller_state_t state;
    double dq; //difference of joint angle from base(qRef) from tdc. it is calcurated by dq = integrate(qd * dt), dq*dt is output of tdc 
    double transition_dq; // for transition. first value is last difference of joint angle from qRef (dq + transition_dq) when state was changed to STOP
    double recovery_dq; // difference of joint angle in 1 cycle to be recoverd
    double min_dq; // min dq when transition
    double max_dq; // max dq when transition
    void setupTwoDofController(double _ke, double _tc, double _dt);
    void setupTwoDofControllerPDModel(double _ke, double _kd, double _tc, double _dt);
    void setupTwoDofControllerDynamicsModel(double _alpha, double _beta, double _ki, double _tc, double _dt);
    double getMotorControllerDq(void); // get according dq according to state
  };
  
  // internal functions
  void setupControllerCommon(std::string _jname, double _dt);
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
