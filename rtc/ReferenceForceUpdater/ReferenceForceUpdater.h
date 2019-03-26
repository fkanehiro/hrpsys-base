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

#include <rtm/idl/BasicDataType.hh>
#include <rtm/idl/ExtendedDataTypes.hh>
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
#include "../TorqueFilter/IIRFilter.h"
#include <boost/shared_ptr.hpp>

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

  bool setReferenceForceUpdaterParam(const std::string& i_name_, const OpenHRP::ReferenceForceUpdaterService::ReferenceForceUpdaterParam& i_param);
  bool getReferenceForceUpdaterParam(const std::string& i_name_, OpenHRP::ReferenceForceUpdaterService::ReferenceForceUpdaterParam_out i_param);
  bool startReferenceForceUpdater(const std::string& i_name_);
  bool stopReferenceForceUpdater(const std::string& i_name_);
  bool startReferenceForceUpdaterNoWait(const std::string& i_name_);
  bool stopReferenceForceUpdaterNoWait(const std::string& i_name_);
  void waitReferenceForceUpdaterTransition(const std::string& i_name_);
  bool getSupportedReferenceForceUpdaterNameSequence(OpenHRP::ReferenceForceUpdaterService::StrSequence_out o_names);
  void getTargetParameters ();
  void calcFootOriginCoords (hrp::Vector3& foot_origin_pos, hrp::Matrix33& foot_origin_rot);
  void updateRefFootOriginExtMoment (const std::string& arm);
  void updateRefObjExtMoment0 (const std::string& arm);
  void updateRefForces (const std::string& arm);
  inline bool eps_eq(const double a, const double b, const double eps = 1e-3) { return std::fabs((a)-(b)) <= eps; };

 protected:
  // Configuration variable declaration
  // <rtc-template block="config_declare">
  
  // </rtc-template>

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
  TimedPoint3D m_diffFootOriginExtMoment;
  InPort<TimedPoint3D> m_diffFootOriginExtMomentIn;

  // </rtc-template>

  // DataOutPort declaration
  // <rtc-template block="outport_declare">
  std::vector<TimedDoubleSeq> m_ref_force_out;
  std::vector<OutPort<TimedDoubleSeq> *> m_ref_forceOut;
  TimedPoint3D m_refFootOriginExtMoment;
  OutPort<TimedPoint3D> m_refFootOriginExtMomentOut;
  TimedBoolean m_refFootOriginExtMomentIsHoldValue;
  OutPort<TimedBoolean> m_refFootOriginExtMomentIsHoldValueOut;

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
  struct ReferenceForceUpdaterParam {
    // Update frequency [Hz]
    double update_freq;
    // Update time ratio \in [0,1]
    double update_time_ratio;
    // P gain
    double p_gain;
    // D gain
    double d_gain;
    // I gain
    double i_gain;
    // Transition time[s]
    double transition_time;
    // Motion direction to update reference force
    hrp::Vector3 motion_dir;
    std::string frame;
    int update_count;
    bool is_active, is_stopping, is_hold_value;
    boost::shared_ptr<FirstOrderLowPassFilter<hrp::Vector3> > act_force_filter;
    void printParam (const std::string print_str)
    {
        std::cerr << "[" << print_str << "]   p_gain = " << p_gain << ", d_gain = " << d_gain << ", i_gain = " << i_gain << std::endl;
        std::cerr << "[" << print_str << "]   update_freq = " << update_freq << "[Hz], update_count = " << update_count << ", update_time_ratio = " << update_time_ratio << ", transition_time = " << transition_time << "[s], cutoff_freq = " << act_force_filter->getCutOffFreq() << "[Hz]" << std::endl;
        std::cerr << "[" << print_str << "]   motion_dir = " << motion_dir.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "    [", "]")) << std::endl;
        std::cerr << "[" << print_str << "]   frame = " << frame << ", is_hold_value = " << (is_hold_value?"true":"false") << std::endl;
    }
    void initializeParam () {
      //params defined in idl
      motion_dir = hrp::Vector3::UnitZ();
      frame="local";
      update_freq = 50; // Hz
      update_time_ratio = 0.5;
      p_gain = 0.02;
      d_gain = 0;
      i_gain = 0;
      transition_time = 1.0;
      //additional params (not defined in idl)
      is_active = false;
      is_stopping = false;
      is_hold_value = false;
    };
    ReferenceForceUpdaterParam () {
      initializeParam();
    };
    ReferenceForceUpdaterParam (const double _dt) {
      initializeParam();
      update_count = round((1/update_freq)/_dt);
      double default_cutoff_freq = 1e8;
      act_force_filter = boost::shared_ptr<FirstOrderLowPassFilter<hrp::Vector3> >(new FirstOrderLowPassFilter<hrp::Vector3>(default_cutoff_freq, _dt, hrp::Vector3::Zero()));
    };
  };
  std::map<std::string, hrp::VirtualForceSensorParam> m_vfs;
  hrp::BodyPtr m_robot;
  double m_dt;
  unsigned int m_debugLevel;
  coil::Mutex m_mutex;
  std::map<std::string, ee_trans> ee_map;
  std::map<std::string, size_t> ee_index_map;
  std::map<std::string, ReferenceForceUpdaterParam> m_RFUParam;
  std::vector<hrp::Vector3> ref_force;
  std::map<std::string, interpolator*> ref_force_interpolator;
  std::map<std::string, interpolator*> transition_interpolator;
  std::vector<double> transition_interpolator_ratio;
  hrp::Matrix33 foot_origin_rot;
  bool use_sh_base_pos_rpy;
  int loop;//counter in onExecute
  const std::string footoriginextmoment_name, objextmoment0_name;
};


extern "C"
{
  void ReferenceForceUpdaterInit(RTC::Manager* manager);
};

#endif // REFERENCEFORCEUPDATOR_COMPONENT_H
