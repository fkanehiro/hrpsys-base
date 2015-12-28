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
  enum motor_model_t {
    TWO_DOF_CONTROLLER,
    TWO_DOF_CONTROLLER_PD_MODEL,
    TWO_DOF_CONTROLLER_DYNAMICS_MODEL
  };

  enum controller_state_t {
    INACTIVE, // dq = 0
    STOP, // resume
    ACTIVE // execute torque control
  };

  MotorTorqueController();
  ~MotorTorqueController(void);

  // for TwoDofController
  MotorTorqueController(std::string _jname, TwoDofController::TwoDofControllerParam &_param);
  void setupController(TwoDofController::TwoDofControllerParam &_param);
  bool getControllerParam(TwoDofController::TwoDofControllerParam &_param);
  bool updateControllerParam(TwoDofController::TwoDofControllerParam &_param);
  // for TwoDofControllerPDModel
  MotorTorqueController(std::string _jname, TwoDofControllerPDModel::TwoDofControllerPDModelParam &_param);
  void setupController(TwoDofControllerPDModel::TwoDofControllerPDModelParam &_param);
  bool getControllerParam(TwoDofControllerPDModel::TwoDofControllerPDModelParam &_param);
  bool updateControllerParam(TwoDofControllerPDModel::TwoDofControllerPDModelParam &_param);
  // for TwoDofControllerDynamicsModel
  MotorTorqueController(std::string _jname, TwoDofControllerDynamicsModel::TwoDofControllerDynamicsModelParam &_param);
  void setupController(TwoDofControllerDynamicsModel::TwoDofControllerDynamicsModelParam &_param);
  bool getControllerParam(TwoDofControllerDynamicsModel::TwoDofControllerDynamicsModelParam &_param);
  bool updateControllerParam(TwoDofControllerDynamicsModel::TwoDofControllerDynamicsModelParam &_param);

  // for normal/emergency torque contorller
  bool enable(void); // enable torque controller (normal controller is not activated but emergency toruqe control may be activated)
  bool disable(void); // disable torque controller (emergency controller is also ignored)

  // for normal torque controller
  void setupMotorControllerControlMinMaxDq(double _min_dq, double _max_dq); // set min/max dq for control
  void setupMotorControllerTransitionMinMaxDq(double _min_transition_dq, double _max_transition_dq); // set min/max dq for transition
  bool activate(void); // set state of torque controller to ACTIVE
  bool deactivate(void); // set state of torque controller to STOP -> INACTIVE
  bool setReferenceTorque(double _tauRef); // set reference torque (does not activate controller)

  double execute(double _tau, double _tauMax); // determine final state and tauRef, then throw tau, tauRef and state to executeControl
  
  // accessor
  motor_model_t getMotorModelType(void);
  std::string getJointName(void);
  controller_state_t getMotorControllerState(void);
  bool isEnabled(void);

  // for debug
  void setErrorPrefix(const std::string& _error_prefix);
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
    double min_dq; // min total dq when control
    double max_dq; // max total dq when control
    double min_transition_dq; // min dq when transition
    double max_transition_dq; // max dq when transition

    // for TwoDofController
    void setupTwoDofController(TwoDofController::TwoDofControllerParam &_param);
    bool getTwoDofControllerParam(TwoDofController::TwoDofControllerParam &_param);
    bool updateTwoDofControllerParam(TwoDofController::TwoDofControllerParam &_param);
    // for TwoDofControllerPDModel
    void setupTwoDofControllerPDModel(TwoDofControllerPDModel::TwoDofControllerPDModelParam &_param);
    bool getTwoDofControllerPDModelParam(TwoDofControllerPDModel::TwoDofControllerPDModelParam &_param);
    bool updateTwoDofControllerPDModelParam(TwoDofControllerPDModel::TwoDofControllerPDModelParam &_param);
    // for TwoDofControllerDynamicsModel
    void setupTwoDofControllerDynamicsModel(TwoDofControllerDynamicsModel::TwoDofControllerDynamicsModelParam &_param);
    bool getTwoDofControllerDynamiccsModelParam(TwoDofControllerDynamicsModel::TwoDofControllerDynamicsModelParam &_param);
    bool updateTwoDofControllerDynamiccsModelParam(TwoDofControllerDynamicsModel::TwoDofControllerDynamicsModelParam &_param);
    double getMotorControllerDq(void); // get according dq according to state
    void setErrorPrefix(const std::string& _error_prefix);
  private:
    bool updateParam(double &_param, const double &_new_value); // update param if new_value is acceptable
    std::string error_prefix;
  };
  
  // internal functions
  void setupControllerCommon(std::string _jname, double _dt);
  void resetMotorControllerVariables(MotorController& _mc); // reset internal torque control parameter  
  void prepareStop(MotorController &_mc);
  void updateController(double _tau, double _tauRef, MotorController& _mc); // execute control and update controller member valiables 
  
  std::string m_joint_name; // joint name which is controled
  motor_model_t m_motor_model_type; // motor model type which is used
  int m_transition_count; // positive value when stopping
  double m_dt; // control term
  double m_current_tau; // current tau (mainly for debug message)
  double m_command_tauRef; // reference tau
  double m_actual_tauRef; // reference tau which is limited or overwritten by emergency (mainly for debug message)
  MotorController m_normalController; // substance of two dof controller
  MotorController m_emergencyController; // overwrite normal controller when emergency
  std::string m_error_prefix; // assumed to be instance name of rtc
  bool m_enable_flag;
};


#endif // MOTOR_TORQUE_CONTROLLER_H
