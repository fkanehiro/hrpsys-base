// -*- C++ -*-
/*!
 * @file  SequencePlayer.h
 * @brief sequence player component
 * @date  $Date$
 *
 * $Id$
 */

#ifndef SEQUENCEPLAYER_H
#define SEQUENCEPLAYER_H

#include <semaphore.h>
#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>
#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/idl/ExtendedDataTypesSkel.h>
#include <hrpModel/Body.h>
#include <hrpModel/Sensor.h>
#include "seqplay.h"

// Service implementation headers
// <rtc-template block="service_impl_h">
#include "SequencePlayerService_impl.h"

// </rtc-template>

// Service Consumer stub headers
// <rtc-template block="consumer_stub_h">

// </rtc-template>

using namespace RTC;

class SequencePlayer
  : public RTC::DataFlowComponentBase
{
 public:
  SequencePlayer(RTC::Manager* manager);
  virtual ~SequencePlayer();

  // The initialize action (on CREATED->ALIVE transition)
  // formaer rtc_init_entry()
 virtual RTC::ReturnCode_t onInitialize();

  // The finalize action (on ALIVE->END transition)
  // formaer rtc_exiting_entry()
  virtual RTC::ReturnCode_t onFinalize();

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
  // virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);

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

  double dt;
  seqplay *player() { return m_seq; }
  hrp::BodyPtr robot() { return m_robot;}  
  void setClearFlag();
  void waitInterpolation();
  bool waitInterpolationOfGroup(const char *gname);
  bool setJointAngle(short id, double angle, double tm);
  bool setJointAngles(const double *angles, double tm);
  bool setJointAngles(const double *angles, const bool *mask, double tm);
  bool setJointAnglesSequence(const OpenHRP::dSequenceSequence angless, const OpenHRP::bSequence& mask, const OpenHRP::dSequence& times);
  bool setJointAnglesSequenceFull(const OpenHRP::dSequenceSequence i_jvss, const OpenHRP::dSequenceSequence i_vels, const OpenHRP::dSequenceSequence i_torques, const OpenHRP::dSequenceSequence i_poss, const OpenHRP::dSequenceSequence i_rpys, const OpenHRP::dSequenceSequence i_accs, const OpenHRP::dSequenceSequence i_zmps, const OpenHRP::dSequenceSequence i_wrenches, const OpenHRP::dSequenceSequence i_optionals, const dSequence i_tms);
  bool clearJointAngles();
  bool setBasePos(const double *pos, double tm);
  bool setBaseRpy(const double *rpy, double tm);
  bool setZmp(const double *zmp, double tm);
  bool setTargetPose(const char* gname, const double *xyz, const double *rpy, double tm, const char* frame_name);
  bool setWrenches(const double *wrenches, double tm);
  void loadPattern(const char *basename, double time); 
  void playPattern(const OpenHRP::dSequenceSequence& pos, const OpenHRP::dSequenceSequence& rpy, const OpenHRP::dSequenceSequence& zmp, const OpenHRP::dSequence& tm);
  bool setInterpolationMode(OpenHRP::SequencePlayerService::interpolationMode i_mode_);
  bool setInitialState(double tm=0.0);
  bool addJointGroup(const char *gname, const OpenHRP::SequencePlayerService::StrSequence& jnames);
  bool removeJointGroup(const char *gname);
  bool setJointAnglesOfGroup(const char *gname, const OpenHRP::dSequence& jvs, double tm);
  bool setJointAnglesSequenceOfGroup(const char *gname, const OpenHRP::dSequenceSequence angless, const OpenHRP::dSequence& times);
    bool clearJointAnglesOfGroup(const char *gname);
  bool playPatternOfGroup(const char *gname, const OpenHRP::dSequenceSequence& pos, const OpenHRP::dSequence& tm);

  void setMaxIKError(double pos, double rot);
  void setMaxIKIteration(short iter);
 protected:
  // Configuration variable declaration
  // <rtc-template block="config_declare">
  
  // </rtc-template>

  // DataInPort declaration
  // <rtc-template block="inport_declare">
  TimedDoubleSeq m_qInit;
  InPort<TimedDoubleSeq> m_qInitIn;
  TimedPoint3D m_basePosInit;
  InPort<TimedPoint3D> m_basePosInitIn;
  TimedOrientation3D m_baseRpyInit;
  InPort<TimedOrientation3D> m_baseRpyInitIn;
  TimedPoint3D m_zmpRefInit;
  InPort<TimedPoint3D> m_zmpRefInitIn;
  
  // </rtc-template>

  // DataOutPort declaration
  // <rtc-template block="outport_declare">
  TimedDoubleSeq m_qRef;
  OutPort<TimedDoubleSeq> m_qRefOut;
  TimedDoubleSeq m_tqRef;
  OutPort<TimedDoubleSeq> m_tqRefOut;
  TimedPoint3D m_zmpRef;
  OutPort<TimedPoint3D> m_zmpRefOut;
  TimedAcceleration3D m_accRef;
  OutPort<TimedAcceleration3D> m_accRefOut;
  TimedPoint3D m_basePos;
  OutPort<TimedPoint3D> m_basePosOut;
  TimedOrientation3D m_baseRpy;
  OutPort<TimedOrientation3D> m_baseRpyOut;
  std::vector<TimedDoubleSeq> m_wrenches;
  std::vector<OutPort<TimedDoubleSeq> *> m_wrenchesOut;
  TimedDoubleSeq m_optionalData;
  OutPort<TimedDoubleSeq> m_optionalDataOut;

  
  // </rtc-template>

  // CORBA Port declaration
  // <rtc-template block="corbaport_declare">
  RTC::CorbaPort m_SequencePlayerServicePort;

  // </rtc-template>

  // Service declaration
  // <rtc-template block="service_declare">
  SequencePlayerService_impl m_service0;

  // </rtc-template>

  // Consumer declaration
  // <rtc-template block="consumer_declare">
  
  // </rtc-template>

 private:
  seqplay *m_seq;
  bool m_clearFlag, m_waitFlag;
  sem_t m_waitSem;
  hrp::BodyPtr m_robot;
  std::string m_gname;
  unsigned int m_debugLevel;
  int dummy;
  size_t optional_data_dim;
  coil::Mutex m_mutex;
  double m_error_pos, m_error_rot;
  short m_iteration;
};


extern "C"
{
  void SequencePlayerInit(RTC::Manager* manager);
};

#endif // SEQUENCEPLAYER_H
