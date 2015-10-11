// -*- C++ -*-
/*!
 * @file  ImpedanceController.h
 * @brief impedance control component
 * @date  $Date$
 *
 * $Id$
 */

#ifndef IMPEDANCE_H
#define IMPEDANCE_H

#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>
#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/idl/ExtendedDataTypesSkel.h>
#include <hrpModel/Body.h>
#include "JointPathEx.h"
#include "RatsMatrix.h"
#include "ImpedanceOutputGenerator.h"
#include "ObjectTurnaroundDetector.h"
// Service implementation headers
// <rtc-template block="service_impl_h">
#include "ImpedanceControllerService_impl.h"

// </rtc-template>

// Service Consumer stub headers
// <rtc-template block="consumer_stub_h">

// </rtc-template>

using namespace RTC;

class ImpedanceController
  : public RTC::DataFlowComponentBase
{
 public:
  ImpedanceController(RTC::Manager* manager);
  virtual ~ImpedanceController();

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

  bool startImpedanceController(const std::string& i_name_);
  bool startImpedanceControllerNoWait(const std::string& i_name_);
  bool stopImpedanceController(const std::string& i_name_);
  bool stopImpedanceControllerNoWait(const std::string& i_name_);
  bool setImpedanceControllerParam(const std::string& i_name_, OpenHRP::ImpedanceControllerService::impedanceParam i_param_);
  bool getImpedanceControllerParam(const std::string& i_name_, OpenHRP::ImpedanceControllerService::impedanceParam& i_param_);
  void waitImpedanceControllerTransition(std::string i_name_);
  void startObjectTurnaroundDetection(const double i_ref_diff_wrench, const double i_max_time, const OpenHRP::ImpedanceControllerService::StrSequence& i_ee_names);
  OpenHRP::ImpedanceControllerService::DetectorMode checkObjectTurnaroundDetection();
  bool setObjectTurnaroundDetectorParam(const OpenHRP::ImpedanceControllerService::objectTurnaroundDetectorParam &i_param_);
  bool getObjectTurnaroundDetectorParam(OpenHRP::ImpedanceControllerService::objectTurnaroundDetectorParam& i_param_);
  bool getObjectForcesMoments(OpenHRP::ImpedanceControllerService::Dbl3Sequence_out o_forces, OpenHRP::ImpedanceControllerService::Dbl3Sequence_out o_moments);

 protected:
  // Configuration variable declaration
  // <rtc-template block="config_declare">
  
  // </rtc-template>

  // DataInPort declaration
  // <rtc-template block="inport_declare">
  TimedDoubleSeq m_qCurrent;
  InPort<TimedDoubleSeq> m_qCurrentIn;
  TimedDoubleSeq m_qRef;
  InPort<TimedDoubleSeq> m_qRefIn;
  TimedPoint3D m_basePos;
  InPort<TimedPoint3D> m_basePosIn;
  TimedOrientation3D m_baseRpy;
  InPort<TimedOrientation3D> m_baseRpyIn;
  std::vector<TimedDoubleSeq> m_force;
  std::vector<InPort<TimedDoubleSeq> *> m_forceIn;
  std::vector<TimedDoubleSeq> m_ref_force;
  std::vector<InPort<TimedDoubleSeq> *> m_ref_forceIn;
  TimedOrientation3D m_rpy;
  InPort<TimedOrientation3D> m_rpyIn;
  
  // </rtc-template>

  // DataOutPort declaration
  // <rtc-template block="outport_declare">
  TimedDoubleSeq m_q;
  OutPort<TimedDoubleSeq> m_qOut;
  
  // </rtc-template>

  // CORBA Port declaration
  // <rtc-template block="corbaport_declare">
  RTC::CorbaPort m_ImpedanceControllerServicePort;

  // </rtc-template>

  // Service declaration
  // <rtc-template block="service_declare">
  ImpedanceControllerService_impl m_service0;

  // </rtc-template>

  // Consumer declaration
  // <rtc-template block="consumer_declare">
  
  // </rtc-template>

 private:

  struct ImpedanceParam : public ImpedanceOutputGenerator {
    std::string sensor_name;
    hrp::Vector3 ref_force, ref_moment;
    double sr_gain, avoid_gain, reference_gain, manipulability_limit;
    int transition_count; // negative value when initing and positive value when deleting
    hrp::dvector transition_joint_q;
    hrp::JointPathExPtr manip;
    bool is_active;

    ImpedanceParam ()
      : ImpedanceOutputGenerator(),
        ref_force(hrp::Vector3::Zero()), ref_moment(hrp::Vector3::Zero()),
        sr_gain(1.0), avoid_gain(0.001), reference_gain(0.01), manipulability_limit(0.1), transition_count(0), is_active(false)
    {};
  };
  struct ee_trans {
    std::string target_name;
    hrp::Vector3 localPos;
    hrp::Matrix33 localR;
  };

  void copyImpedanceParam (OpenHRP::ImpedanceControllerService::impedanceParam& i_param_, const ImpedanceParam& param);
  void updateRootLinkPosRot (TimedOrientation3D tmprpy);
  void calcForceMoment();

  std::map<std::string, ImpedanceParam> m_impedance_param;
  std::map<std::string, ee_trans> ee_map;
  std::map<std::string, hrp::VirtualForceSensorParam> m_vfs;
  std::map<std::string, hrp::Vector3> abs_forces, abs_moments, abs_ref_forces, abs_ref_moments;
  boost::shared_ptr<ObjectTurnaroundDetector > otd;
  std::vector<std::string> otd_sensor_names;
  hrp::Vector3 otd_axis;
  double m_dt;
  hrp::BodyPtr m_robot;
  coil::Mutex m_mutex;
  hrp::dvector qrefv;
  unsigned int m_debugLevel;
  int dummy;
  int loop;
  bool use_sh_base_pos_rpy;
};


extern "C"
{
  void ImpedanceControllerInit(RTC::Manager* manager);
};

#endif // IMPEDANCE_H
