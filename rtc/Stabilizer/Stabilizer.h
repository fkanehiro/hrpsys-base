// -*- C++ -*-
/*!
 * @file  Stabilizer.h
 * @brief stabilizer component
 * @date  $Date$
 *
 * $Id$
 */

#ifndef STABILIZER_COMPONENT_H
#define STABILIZER_COMPONENT_H

#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>
#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/idl/ExtendedDataTypesSkel.h>
#include <hrpModel/Body.h>

// Service implementation headers
// <rtc-template block="service_impl_h">
#include "StabilizerService_impl.h"
#include "TwoDofController.h"
#include "../ImpedanceController/JointPathEx.h"
#include "../ImpedanceController/RatsMatrix.h"

// </rtc-template>

// Service Consumer stub headers
// <rtc-template block="consumer_stub_h">

// </rtc-template>

/**
   \brief sample RT component which has one data input port and one data output port
 */
#define ST_NUM_LEGS 2
#define ST_MAX_DQ (10 * M_PI / 180)

class Stabilizer
  : public RTC::DataFlowComponentBase
{
 public:
  /**
     \brief Constructor
     \param manager pointer to the Manager
  */
  Stabilizer(RTC::Manager* manager);
  /**
     \brief Destructor
  */
  virtual ~Stabilizer();

  // The initialize action (on CREATED->ALIVE transition)
  // formaer rtc_init_entry()
  virtual RTC::ReturnCode_t onInitialize();

  // The finalize action (on ALIVE->END transition)
  // formaer rtc_exiting_entry()
  // virtual RTC::ReturnCode_t onFinalize();

  // The startup action when ExecutionContext startup
  // former rtc_starting_entry()
  // virtual RTC::ReturnCode_t onStartup(RTC::UniqueId ec_id);

  // The shutdown action when ExecutionContext stop
  // former rtc_stopping_entry()
  // virtual RTC::ReturnCode_t onShutdown(RTC::UniqueId ec_id);

  // The activated action (Active state entry action)
  // former rtc_active_entry()
  virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);

  // The deactivated action (Active state exit action)
  // former rtc_active_exit()
  virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);

  // The execution action that is invoked periodically
  // former rtc_active_do()
  virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

  // The aborting action when main logic error occurred.
  // former rtc_aborting_entry()
  // virtual RTC::ReturnCode_t onAborting(RTC::UniqueId ec_id);

  // The error action in ERROR state
  // former rtc_error_do()
  // virtual RTC::ReturnCode_t onError(RTC::UniqueId ec_id);

  // The reset action that is invoked resetting
  // This is same but different the former rtc_init_entry()
  // virtual RTC::ReturnCode_t onReset(RTC::UniqueId ec_id);

  // The state update action that is invoked after onExecute() action
  // no corresponding operation exists in OpenRTm-aist-0.2.0
  // virtual RTC::ReturnCode_t onStateUpdate(RTC::UniqueId ec_id);

  // The action that is invoked when execution context's rate is changed
  // no corresponding operation exists in OpenRTm-aist-0.2.0
  // virtual RTC::ReturnCode_t onRateChanged(RTC::UniqueId ec_id);

  void startStabilizer(void);
  void stopStabilizer(void);
  void getCurrentParameters ();
  void getTargetParameters ();
  void sync_2_st ();
  void sync_2_idle();
  bool calcZMP(hrp::Vector3& ret_zmp);
  void calcRUNST();
  void calcTPCC();
  void setParameter(const OpenHRP::StabilizerService::stParam& i_stp);
  void waitSTTransition();
  // funcitons for calc final torque output
  void calcContactMatrix (hrp::dmatrix& tm, const std::vector<hrp::Vector3>& contact_p);
  void calcTorque ();

 protected:
  // Configuration variable declaration
  // <rtc-template block="config_declare">
  
  // </rtc-template>
  RTC::TimedDoubleSeq m_qCurrent;
  RTC::TimedDoubleSeq m_qRef;
  RTC::TimedDoubleSeq m_tau;
  RTC::TimedOrientation3D m_rpy;
  // RTC::TimedDoubleSeq m_forceR, m_forceL;
  RTC::TimedDoubleSeq m_force[2];
  RTC::TimedPoint3D m_zmpRef;
  RTC::TimedPoint3D m_basePos;
  RTC::TimedOrientation3D m_baseRpy;
  
  // DataInPort declaration
  // <rtc-template block="inport_declare">
  RTC::InPort<RTC::TimedDoubleSeq> m_qCurrentIn;
  RTC::InPort<RTC::TimedDoubleSeq> m_qRefIn;
  RTC::InPort<RTC::TimedOrientation3D> m_rpyIn;
  RTC::InPort<RTC::TimedDoubleSeq> m_forceRIn;
  RTC::InPort<RTC::TimedDoubleSeq> m_forceLIn;
  RTC::InPort<RTC::TimedPoint3D> m_zmpRefIn;
  RTC::InPort<RTC::TimedPoint3D> m_basePosIn;
  RTC::InPort<RTC::TimedOrientation3D> m_baseRpyIn;
  
  // </rtc-template>

  // DataOutPort declaration
  // <rtc-template block="outport_declare">
  RTC::OutPort<RTC::TimedDoubleSeq> m_qRefOut;
  RTC::OutPort<RTC::TimedDoubleSeq> m_tauOut;
  
  // </rtc-template>

  // CORBA Port declaration
  // <rtc-template block="corbaport_declare">
  
  // </rtc-template>

  // Service declaration
  // <rtc-template block="service_declare">
  RTC::CorbaPort m_StabilizerServicePort;
  
  // </rtc-template>

  // Consumer declaration
  // <rtc-template block="consumer_declare">
  StabilizerService_impl m_service0;
  
  // </rtc-template>

 private:
  // constant defines
  enum {
    ST_LEFT = 0,
    ST_RIGHT = 1
  };
  enum {
    ST_X = 0,
    ST_Y = 1
  };
  enum cmode {MODE_IDLE, MODE_AIR, MODE_ST, MODE_SYNC_TO_IDLE, MODE_SYNC_TO_AIR} control_mode;
  // members
  hrp::JointPathExPtr manip2[2];
  hrp::BodyPtr m_robot;
  unsigned int m_debugLevel;
  hrp::dvector transition_joint_q, qorg, qrefv;
  std::vector<std::string> sensor_names;
  double dt;
  int transition_count, loop;
  bool is_legged_robot, is_qCurrent;
  hrp::Vector3 current_root_p;
  hrp::Matrix33 current_root_R;
  hrp::Matrix33 target_root_R;
  hrp::Vector3 target_foot_p[2];
  hrp::Matrix33 target_foot_R[2];
  hrp::Vector3 refzmp, refcog, refcog_vel;
  // TPCC
  double k_tpcc_p[2], k_tpcc_x[2], d_rpy[2], k_brot_p[2], k_brot_tc[2];
  hrp::Vector3 act_zmp, prefcog;
  // RUN ST
  TwoDofController m_tau_x[ST_NUM_LEGS], m_tau_y[ST_NUM_LEGS], m_f_z;
  hrp::Vector3 pdr;
  double m_torque_k[2], m_torque_d[2]; // 3D-LIP parameters (0: x, 1: y)
  double pangx_ref, pangy_ref, pangx, pangy;
  double k_run_b[2], d_run_b[2];
  double rdx, rdy, rx, ry;
};


extern "C"
{
  void StabilizerInit(RTC::Manager* manager);
};

#endif // STABILIZER_COMPONENT_H
