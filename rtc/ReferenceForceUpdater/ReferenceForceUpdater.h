// -*- C++ -*-
/*!
 * @file  ReferenceForceUpdater.h
 * @brief ReferenceForceUpdater
 * @date  $Date$
 *
 * $Id$
 */

#ifndef REFERENCEFORCEUPDATOR_COMPONENT_H
#define REFERENCEFORCEUPDATOR_COMPONENT_H

#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>
#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/idl/ExtendedDataTypesSkel.h>
#include <hrpModel/Body.h>
#include "../ImpedanceController/JointPathEx.h"
#include "../ImpedanceController/RatsMatrix.h"
#include "../SequencePlayer/interpolator.h"
// #include "ImpedanceOutputGenerator.h"
// #include "ObjectTurnaroundDetector.h"
// Service implementation headers
// <rtc-template block="service_impl_h">
#include "ReferenceForceUpdaterService_impl.h"

// </rtc-template>

// Service Consumer stub headers
// <rtc-template block="consumer_stub_h">

// </rtc-template>

using namespace RTC;

/**
   \brief sample RT component which has one data input port and one data output port
 */
class ReferenceForceUpdater
  : public RTC::DataFlowComponentBase
{
 public:
  /**
     \brief Constructor
     \param manager pointer to the Manager
  */
  ReferenceForceUpdater(RTC::Manager* manager);
  /**
     \brief Destructor
  */
  virtual ~ReferenceForceUpdater();

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

  bool setReferenceForceUpdaterParam(const OpenHRP::ReferenceForceUpdaterService::ReferenceForceUpdaterParam& i_param);
  bool getReferenceForceUpdaterParam(OpenHRP::ReferenceForceUpdaterService::ReferenceForceUpdaterParam_out i_param);
  bool startReferenceForceUpdater();
  bool stopReferenceForceUpdater();

 protected:
  // Configuration variable declaration
  // <rtc-template block="config_declare">
  
  // </rtc-template>

  TimedDouble m_data;

  // DataInPort declaration
  // <rtc-template block="inport_declare">
  TimedDoubleSeq m_qRef;
  InPort<TimedDoubleSeq> m_qRefIn;
  TimedPoint3D m_basePos;
  InPort<TimedPoint3D> m_basePosIn;
  TimedOrientation3D m_baseRpy;
  InPort<TimedOrientation3D> m_baseRpyIn;
  std::vector<TimedDoubleSeq> m_force;
  std::vector<InPort<TimedDoubleSeq> *> m_forceIn;
  std::vector<TimedDoubleSeq> m_ref_force_in;
  std::vector<InPort<TimedDoubleSeq> *> m_ref_forceIn;
  TimedOrientation3D m_rpy;
  InPort<TimedOrientation3D> m_rpyIn;

  // </rtc-template>

  // DataOutPort declaration
  // <rtc-template block="outport_declare">
  std::vector<TimedDoubleSeq> m_ref_force_out;
  std::vector<OutPort<TimedDoubleSeq> *> m_ref_forceOut;

  // </rtc-template>

  // CORBA Port declaration
  // <rtc-template block="corbaport_declare">
  
  // </rtc-template>

  // Service declaration
  // <rtc-template block="service_declare">
  RTC::CorbaPort m_ReferenceForceUpdaterServicePort;
  
  // </rtc-template>

  // Consumer declaration
  // <rtc-template block="consumer_declare">
  ReferenceForceUpdaterService_impl m_ReferenceForceUpdaterService;
  
  // </rtc-template>

 private:
  struct ee_trans {
    std::string target_name, sensor_name;
    hrp::Vector3 localPos;
    hrp::Matrix33 localR;
  };
  std::map<std::string, hrp::VirtualForceSensorParam> m_vfs;
  hrp::BodyPtr m_robot;
  double m_dt;
  unsigned int m_debugLevel;
  coil::Mutex m_mutex;
  std::map<std::string, ee_trans> ee_map;
  std::map<std::string, size_t> ee_index_map;
  std::vector<hrp::Vector3> ref_force;
  std::map<std::string, interpolator*> ref_force_interpolator;
  interpolator* transition_interpolator;
  double update_freq, p_gain, d_gain, i_gain, update_time_ratio;
  hrp::Vector3 motion_dir;
  std::string arm;
  bool use_sh_base_pos_rpy, is_active, is_stopping;
  int loop;//counter in onExecute
  int update_count;
};


extern "C"
{
  void ReferenceForceUpdaterInit(RTC::Manager* manager);
};

#endif // REFERENCEFORCEUPDATOR_COMPONENT_H
