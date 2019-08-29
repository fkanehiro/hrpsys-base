// -*- C++ -*-
/*!
 * @file  Stabilizer.cpp
 * @brief stabilizer filter
 * $Date$
 *
 * $Id$
 */

#include <rtm/CorbaNaming.h>
#include <hrpModel/Link.h>
#include <hrpModel/Sensor.h>
#include <hrpModel/ModelLoaderUtil.h>
#include "Stabilizer.h"
#include "hrpsys/util/VectorConvert.h"
#include <math.h>
#include <boost/lambda/lambda.hpp>

typedef coil::Guard<coil::Mutex> Guard;

#ifndef deg2rad
#define deg2rad(x) ((x) * M_PI / 180.0)
#endif
#ifndef rad2deg
#define rad2deg(rad) (rad * 180 / M_PI)
#endif

// Module specification
// <rtc-template block="module_spec">
static const char* stabilizer_spec[] =
  {
    "implementation_id", "Stabilizer",
    "type_name",         "Stabilizer",
    "description",       "stabilizer",
    "version",           HRPSYS_PACKAGE_VERSION,
    "vendor",            "AIST",
    "category",          "example",
    "activity_type",     "DataFlowComponent",
    "max_instance",      "10",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    "conf.default.debugLevel", "0",
    ""
  };
// </rtc-template>

static std::ostream& operator<<(std::ostream& os, const struct RTC::Time &tm)
{
    int pre = os.precision();
    os.setf(std::ios::fixed);
    os << std::setprecision(6)
       << (tm.sec + tm.nsec/1e9)
       << std::setprecision(pre);
    os.unsetf(std::ios::fixed);
    return os;
}

static double switching_inpact_absorber(double force, double lower_th, double upper_th);

Stabilizer::Stabilizer(RTC::Manager* manager)
  : RTC::DataFlowComponentBase(manager),
    // <rtc-template block="initializer">
    m_qCurrentIn("qCurrent", m_qCurrent),
    m_qRefIn("qRef", m_qRef),
    m_rpyIn("rpy", m_rpy),
    m_zmpRefIn("zmpRef", m_zmpRef),
    m_StabilizerServicePort("StabilizerService"),
    m_basePosIn("basePosIn", m_basePos),
    m_baseRpyIn("baseRpyIn", m_baseRpy),
    m_contactStatesIn("contactStates", m_contactStates),
    m_toeheelRatioIn("toeheelRatio", m_toeheelRatio),
    m_controlSwingSupportTimeIn("controlSwingSupportTime", m_controlSwingSupportTime),
    m_qRefSeqIn("qRefSeq", m_qRefSeq),
    m_walkingStatesIn("walkingStates", m_walkingStates),
    m_sbpCogOffsetIn("sbpCogOffset", m_sbpCogOffset),
    m_qRefOut("q", m_qRef),
    m_tauOut("tau", m_tau),
    m_zmpOut("zmp", m_zmp),
    m_refCPOut("refCapturePoint", m_refCP),
    m_actCPOut("actCapturePoint", m_actCP),
    m_diffCPOut("diffCapturePoint", m_diffCP),
    m_diffFootOriginExtMomentOut("diffFootOriginExtMoment", m_diffFootOriginExtMoment),
    m_actContactStatesOut("actContactStates", m_actContactStates),
    m_COPInfoOut("COPInfo", m_COPInfo),
    m_emergencySignalOut("emergencySignal", m_emergencySignal),
    // for debug output
    m_originRefZmpOut("originRefZmp", m_originRefZmp),
    m_originRefCogOut("originRefCog", m_originRefCog),
    m_originRefCogVelOut("originRefCogVel", m_originRefCogVel),
    m_originNewZmpOut("originNewZmp", m_originNewZmp),
    m_originActZmpOut("originActZmp", m_originActZmp),
    m_originActCogOut("originActCog", m_originActCog),
    m_originActCogVelOut("originActCogVel", m_originActCogVel),
    m_actBaseRpyOut("actBaseRpy", m_actBaseRpy),
    m_currentBasePosOut("currentBasePos", m_currentBasePos),
    m_currentBaseRpyOut("currentBaseRpy", m_currentBaseRpy),
    m_allRefWrenchOut("allRefWrench", m_allRefWrench),
    m_allEECompOut("allEEComp", m_allEEComp),
    m_debugDataOut("debugData", m_debugData),
    control_mode(MODE_IDLE),
    st_algorithm(OpenHRP::StabilizerService::TPCC),
    emergency_check_mode(OpenHRP::StabilizerService::NO_CHECK),
    szd(NULL),
    // </rtc-template>
    m_debugLevel(0)
{
  m_service0.stabilizer(this);
}

Stabilizer::~Stabilizer()
{
}

RTC::ReturnCode_t Stabilizer::onInitialize()
{
  std::cerr << "[" << m_profile.instance_name << "] onInitialize()" << std::endl;
  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  bindParameter("debugLevel", m_debugLevel, "0");
  
  // </rtc-template>

  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("qCurrent", m_qCurrentIn);
  addInPort("qRef", m_qRefIn);
  addInPort("rpy", m_rpyIn);
  addInPort("zmpRef", m_zmpRefIn);
  addInPort("basePosIn", m_basePosIn);
  addInPort("baseRpyIn", m_baseRpyIn);
  addInPort("contactStates", m_contactStatesIn);
  addInPort("toeheelRatio", m_toeheelRatioIn);
  addInPort("controlSwingSupportTime", m_controlSwingSupportTimeIn);
  addInPort("qRefSeq", m_qRefSeqIn);
  addInPort("walkingStates", m_walkingStatesIn);
  addInPort("sbpCogOffset", m_sbpCogOffsetIn);

  // Set OutPort buffer
  addOutPort("q", m_qRefOut);
  addOutPort("tau", m_tauOut);
  addOutPort("zmp", m_zmpOut);
  addOutPort("refCapturePoint", m_refCPOut);
  addOutPort("actCapturePoint", m_actCPOut);
  addOutPort("diffCapturePoint", m_diffCPOut);
  addOutPort("diffStaticBalancePointOffset", m_diffFootOriginExtMomentOut);
  addOutPort("actContactStates", m_actContactStatesOut);
  addOutPort("COPInfo", m_COPInfoOut);
  addOutPort("emergencySignal", m_emergencySignalOut);
  // for debug output
  addOutPort("originRefZmp", m_originRefZmpOut);
  addOutPort("originRefCog", m_originRefCogOut);
  addOutPort("originRefCogVel", m_originRefCogVelOut);
  addOutPort("originNewZmp", m_originNewZmpOut);
  addOutPort("originActZmp", m_originActZmpOut);
  addOutPort("originActCog", m_originActCogOut);
  addOutPort("originActCogVel", m_originActCogVelOut);
  addOutPort("actBaseRpy", m_actBaseRpyOut);
  addOutPort("currentBasePos", m_currentBasePosOut);
  addOutPort("currentBaseRpy", m_currentBaseRpyOut);
  addOutPort("allRefWrench", m_allRefWrenchOut);
  addOutPort("allEEComp", m_allEECompOut);
  addOutPort("debugData", m_debugDataOut);
  
  // Set service provider to Ports
  m_StabilizerServicePort.registerProvider("service0", "StabilizerService", m_service0);
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  addPort(m_StabilizerServicePort);
  
  // </rtc-template>
  RTC::Properties& prop = getProperties();
  coil::stringTo(dt, prop["dt"].c_str());

  // parameters for corba
  RTC::Manager& rtcManager = RTC::Manager::instance();
  std::string nameServer = rtcManager.getConfig()["corba.nameservers"];
  int comPos = nameServer.find(",");
  if (comPos < 0){
    comPos = nameServer.length();
  }
  nameServer = nameServer.substr(0, comPos);
  RTC::CorbaNaming naming(rtcManager.getORB(), nameServer.c_str());

  // parameters for internal robot model
  m_robot = hrp::BodyPtr(new hrp::Body());
  if (!loadBodyFromModelLoader(m_robot, prop["model"].c_str(), 
                               CosNaming::NamingContext::_duplicate(naming.getRootContext())
                               )){
    std::cerr << "[" << m_profile.instance_name << "]failed to load model[" << prop["model"] << "]" << std::endl;
    return RTC::RTC_ERROR;
  }

  // Setting for wrench data ports (real + virtual)
  std::vector<std::string> force_sensor_names;

  // Find names for real force sensors
  int npforce = m_robot->numSensors(hrp::Sensor::FORCE);
  for (unsigned int i=0; i<npforce; ++i) {
      force_sensor_names.push_back(m_robot->sensor(hrp::Sensor::FORCE, i)->name);
  }

  // load virtual force sensors
  readVirtualForceSensorParamFromProperties(m_vfs, m_robot, prop["virtual_force_sensor"], std::string(m_profile.instance_name));
  int nvforce = m_vfs.size();
  for (unsigned int i=0; i<nvforce; ++i) {
      for ( std::map<std::string, hrp::VirtualForceSensorParam>::iterator it = m_vfs.begin(); it != m_vfs.end(); it++ ) {
          if (it->second.id == i) {
              force_sensor_names.push_back(it->first);
          }
      }
  }

  int nforce = npforce + nvforce;
  m_wrenches.resize(nforce);
  m_wrenchesIn.resize(nforce);
  m_ref_wrenches.resize(nforce);
  m_ref_wrenchesIn.resize(nforce);
  m_limbCOPOffset.resize(nforce);
  m_limbCOPOffsetIn.resize(nforce);
  std::cerr << "[" << m_profile.instance_name << "] force sensor ports (" << npforce << ")" << std::endl;
  for (unsigned int i=0; i<nforce; ++i) {
      std::string force_sensor_name = force_sensor_names[i];
      // actual inport
      m_wrenchesIn[i] = new RTC::InPort<RTC::TimedDoubleSeq>(force_sensor_name.c_str(), m_wrenches[i]);
      m_wrenches[i].data.length(6);
      registerInPort(force_sensor_name.c_str(), *m_wrenchesIn[i]);
      // referecen inport
      m_ref_wrenchesIn[i] = new RTC::InPort<RTC::TimedDoubleSeq>(std::string(force_sensor_name+"Ref").c_str(), m_ref_wrenches[i]);
      m_ref_wrenches[i].data.length(6);
      registerInPort(std::string(force_sensor_name+"Ref").c_str(), *m_ref_wrenchesIn[i]);
      std::cerr << "[" << m_profile.instance_name << "]   name = " << force_sensor_name << std::endl;
  }
  std::cerr << "[" << m_profile.instance_name << "] limbCOPOffset ports (" << npforce << ")" << std::endl;
  for (unsigned int i=0; i<nforce; ++i) {
      std::string force_sensor_name = force_sensor_names[i];
      std::string nm("limbCOPOffset_"+force_sensor_name);
      m_limbCOPOffsetIn[i] = new RTC::InPort<RTC::TimedPoint3D>(nm.c_str(), m_limbCOPOffset[i]);
      registerInPort(nm.c_str(), *m_limbCOPOffsetIn[i]);
      std::cerr << "[" << m_profile.instance_name << "]   name = " << nm << std::endl;
  }

  // setting from conf file
  // rleg,TARGET_LINK,BASE_LINK,x,y,z,rx,ry,rz,rth #<=pos + rot (axis+angle)
  coil::vstring end_effectors_str = coil::split(prop["end_effectors"], ",");
  if (end_effectors_str.size() > 0) {
    size_t prop_num = 10;
    size_t num = end_effectors_str.size()/prop_num;
    for (size_t i = 0; i < num; i++) {
      std::string ee_name, ee_target, ee_base;
      coil::stringTo(ee_name, end_effectors_str[i*prop_num].c_str());
      coil::stringTo(ee_target, end_effectors_str[i*prop_num+1].c_str());
      coil::stringTo(ee_base, end_effectors_str[i*prop_num+2].c_str());
      STIKParam ikp;
      for (size_t j = 0; j < 3; j++) {
        coil::stringTo(ikp.localp(j), end_effectors_str[i*prop_num+3+j].c_str());
      }
      double tmpv[4];
      for (int j = 0; j < 4; j++ ) {
        coil::stringTo(tmpv[j], end_effectors_str[i*prop_num+6+j].c_str());
      }
      ikp.localR = Eigen::AngleAxis<double>(tmpv[3], hrp::Vector3(tmpv[0], tmpv[1], tmpv[2])).toRotationMatrix(); // rotation in VRML is represented by axis + angle
      ikp.target_name = ee_target;
      ikp.ee_name = ee_name;
      {
          bool is_ee_exists = false;
          for (size_t j = 0; j < npforce; j++) {
              hrp::Sensor* sensor = m_robot->sensor(hrp::Sensor::FORCE, j);
              hrp::Link* alink = m_robot->link(ikp.target_name);
              while (alink != NULL && alink->name != ee_base && !is_ee_exists) {
                  if ( alink->name == sensor->link->name ) {
                      is_ee_exists = true;
                      ikp.sensor_name = sensor->name;
                  }
                  alink = alink->parent;
              }
          }
      }
      ikp.avoid_gain = 0.001;
      ikp.reference_gain = 0.01;
      ikp.ik_loop_count = 3;
      // For swing ee modification
      ikp.target_ee_diff_p = hrp::Vector3::Zero();
      ikp.target_ee_diff_r = hrp::Matrix33::Identity();
      ikp.d_rpy_swing = hrp::Vector3::Zero();
      ikp.d_pos_swing = hrp::Vector3::Zero();
      ikp.target_ee_diff_p_filter = boost::shared_ptr<FirstOrderLowPassFilter<hrp::Vector3> >(new FirstOrderLowPassFilter<hrp::Vector3>(50.0, dt, hrp::Vector3::Zero())); // [Hz]
      ikp.target_ee_diff_r_filter = boost::shared_ptr<FirstOrderLowPassFilter<hrp::Vector3> >(new FirstOrderLowPassFilter<hrp::Vector3>(50.0, dt, hrp::Vector3::Zero())); // [Hz]
      ikp.prev_d_pos_swing = hrp::Vector3::Zero();
      ikp.prev_d_rpy_swing = hrp::Vector3::Zero();
      //
      stikp.push_back(ikp);
      jpe_v.push_back(hrp::JointPathExPtr(new hrp::JointPathEx(m_robot, m_robot->link(ee_base), m_robot->link(ee_target), dt, false, std::string(m_profile.instance_name))));
      // Fix for toe joint
      if (ee_name.find("leg") != std::string::npos && jpe_v.back()->numJoints() == 7) { // leg and has 7dof joint (6dof leg +1dof toe)
          std::vector<double> optw;
          for (int j = 0; j < jpe_v.back()->numJoints(); j++ ) {
              if ( j == jpe_v.back()->numJoints()-1 ) optw.push_back(0.0);
              else optw.push_back(1.0);
          }
          jpe_v.back()->setOptionalWeightVector(optw);
      }
      target_ee_p.push_back(hrp::Vector3::Zero());
      target_ee_R.push_back(hrp::Matrix33::Identity());
      act_ee_p.push_back(hrp::Vector3::Zero());
      act_ee_R.push_back(hrp::Matrix33::Identity());
      projected_normal.push_back(hrp::Vector3::Zero());
      act_force.push_back(hrp::Vector3::Zero());
      ref_force.push_back(hrp::Vector3::Zero());
      ref_moment.push_back(hrp::Vector3::Zero());
      contact_states_index_map.insert(std::pair<std::string, size_t>(ee_name, i));
      is_ik_enable.push_back( (ee_name.find("leg") != std::string::npos ? true : false) ); // Hands ik => disabled, feet ik => enabled, by default
      is_feedback_control_enable.push_back( (ee_name.find("leg") != std::string::npos ? true : false) ); // Hands feedback control => disabled, feet feedback control => enabled, by default
      is_zmp_calc_enable.push_back( (ee_name.find("leg") != std::string::npos ? true : false) ); // To zmp calculation, hands are disabled and feet are enabled, by default
      std::cerr << "[" << m_profile.instance_name << "] End Effector [" << ee_name << "]" << std::endl;
      std::cerr << "[" << m_profile.instance_name << "]   target = " << m_robot->link(ikp.target_name)->name << ", base = " << ee_base << ", sensor_name = " << ikp.sensor_name << std::endl;
      std::cerr << "[" << m_profile.instance_name << "]   offset_pos = " << ikp.localp.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "    [", "]")) << "[m]" << std::endl;
      prev_act_force_z.push_back(0.0);
    }
    m_contactStates.data.length(num);
    m_toeheelRatio.data.length(num);
    m_will_fall_counter.resize(num);
  }

  std::vector<std::pair<hrp::Link*, hrp::Link*> > interlocking_joints;
  readInterlockingJointsParamFromProperties(interlocking_joints, m_robot, prop["interlocking_joints"], std::string(m_profile.instance_name));
  if (interlocking_joints.size() > 0) {
      for (size_t i = 0; i < jpe_v.size(); i++) {
          std::cerr << "[" << m_profile.instance_name << "] Interlocking Joints for [" << stikp[i].ee_name << "]" << std::endl;
          jpe_v[i]->setInterlockingJointPairIndices(interlocking_joints, std::string(m_profile.instance_name));
      }
  }


  // parameters for TPCC
  act_zmp = hrp::Vector3::Zero();
  for (int i = 0; i < 2; i++) {
    k_tpcc_p[i] = 0.2;
    k_tpcc_x[i] = 4.0;
    k_brot_p[i] = 0.1;
    k_brot_tc[i] = 1.5;
  }
  // parameters for EEFM
  double k_ratio = 0.9;
  for (int i = 0; i < 2; i++) {
    eefm_k1[i] = -1.41429*k_ratio;
    eefm_k2[i] = -0.404082*k_ratio;
    eefm_k3[i] = -0.18*k_ratio;
    eefm_body_attitude_control_gain[i] = 0.5;
    eefm_body_attitude_control_time_const[i] = 1e5;
  }
  for (size_t i = 0; i < stikp.size(); i++) {
      STIKParam& ikp = stikp[i];
      hrp::Link* root = m_robot->link(ikp.target_name);
      ikp.eefm_rot_damping_gain = hrp::Vector3(20*5, 20*5, 1e5);
      ikp.eefm_rot_time_const = hrp::Vector3(1.5, 1.5, 1.5);
      ikp.eefm_rot_compensation_limit = deg2rad(10.0);
      ikp.eefm_swing_rot_spring_gain = hrp::Vector3(0.0, 0.0, 0.0);
      ikp.eefm_swing_rot_time_const = hrp::Vector3(1.5, 1.5, 1.5);
      ikp.eefm_pos_damping_gain = hrp::Vector3(3500*10, 3500*10, 3500);
      ikp.eefm_pos_time_const_support = hrp::Vector3(1.5, 1.5, 1.5);
      ikp.eefm_pos_compensation_limit = 0.025;
      ikp.eefm_swing_pos_spring_gain = hrp::Vector3(0.0, 0.0, 0.0);
      ikp.eefm_swing_pos_time_const = hrp::Vector3(1.5, 1.5, 1.5);
      ikp.eefm_ee_moment_limit = hrp::Vector3(1e4, 1e4, 1e4); // Default limit [Nm] is too large. Same as no limit.
      if (ikp.ee_name.find("leg") == std::string::npos) { // Arm default
          ikp.eefm_ee_forcemoment_distribution_weight = Eigen::Matrix<double, 6,1>::Zero();
      } else { // Leg default
          for (size_t j = 0; j < 3; j++) {
              ikp.eefm_ee_forcemoment_distribution_weight[j] = 1; // Force
              ikp.eefm_ee_forcemoment_distribution_weight[j+3] = 1e-2; // Moment
          }
      }
      ikp.max_limb_length = 0.0;
      while (!root->isRoot()) {
        ikp.max_limb_length += root->b.norm();
        ikp.parent_name = root->name;
        root = root->parent;
      }
      ikp.limb_length_margin = 0.13;
      ikp.support_time = 0.0;
  }
  eefm_swing_rot_damping_gain = hrp::Vector3(20*5, 20*5, 1e5);
  eefm_swing_pos_damping_gain = hrp::Vector3(33600, 33600, 7000);
  eefm_pos_time_const_swing = 0.08;
  eefm_pos_transition_time = 0.01;
  eefm_pos_margin_time = 0.02;
  eefm_zmp_delay_time_const[0] = eefm_zmp_delay_time_const[1] = 0.055;
  //eefm_leg_inside_margin = 0.065; // [m]
  //eefm_leg_front_margin = 0.05;
  //eefm_leg_rear_margin = 0.05;
  //fm_wrench_alpha_blending = 1.0; // fz_alpha
  eefm_gravitational_acceleration = 9.80665; // [m/s^2]
  cop_check_margin = 20.0*1e-3; // [m]
  cp_check_margin.resize(4, 30*1e-3); // [m]
  cp_offset = hrp::Vector3(0.0, 0.0, 0.0); // [m]
  tilt_margin.resize(2, 30 * M_PI / 180); // [rad]
  contact_decision_threshold = 50; // [N]
  eefm_use_force_difference_control = true;
  eefm_use_swing_damping = false;
  eefm_swing_damping_force_thre.resize(3, 300);
  eefm_swing_damping_moment_thre.resize(3, 15);
  initial_cp_too_large_error = true;
  is_walking = false;
  is_estop_while_walking = false;
  sbp_cog_offset = hrp::Vector3(0.0, 0.0, 0.0);
  use_limb_stretch_avoidance = false;
  use_zmp_truncation = false;
  limb_stretch_avoidance_time_const = 1.5;
  limb_stretch_avoidance_vlimit[0] = -100 * 1e-3 * dt; // lower limit
  limb_stretch_avoidance_vlimit[1] = 50 * 1e-3 * dt; // upper limit
  root_rot_compensation_limit[0] = root_rot_compensation_limit[1] = deg2rad(90.0);
  detection_count_to_air = static_cast<int>(0.0 / dt);

  // parameters for RUNST
  double ke = 0, tc = 0;
  for (int i = 0; i < 2; i++) {
    m_tau_x[i].setup(ke, tc, dt);
    m_tau_x[i].setErrorPrefix(std::string(m_profile.instance_name));
    m_tau_y[i].setup(ke, tc, dt);
    m_tau_y[i].setErrorPrefix(std::string(m_profile.instance_name));
    m_f_z.setup(ke, tc, dt);
    m_f_z.setErrorPrefix(std::string(m_profile.instance_name));
  }
  pangx_ref = pangy_ref = pangx = pangy = 0;
  rdx = rdy = rx = ry = 0;
  pdr = hrp::Vector3::Zero();

  // Check is legged robot or not
  is_legged_robot = false;
  for (size_t i = 0; i < stikp.size(); i++) {
      if (stikp[i].ee_name.find("leg") == std::string::npos) continue;
      hrp::Sensor* sen= m_robot->sensor<hrp::ForceSensor>(stikp[i].sensor_name);
      if ( sen != NULL ) is_legged_robot = true;
  }
  is_emergency = false;
  reset_emergency_flag = false;

  m_qCurrent.data.length(m_robot->numJoints());
  m_qRef.data.length(m_robot->numJoints());
  m_tau.data.length(m_robot->numJoints());
  transition_joint_q.resize(m_robot->numJoints());
  qorg.resize(m_robot->numJoints());
  qrefv.resize(m_robot->numJoints());
  transition_count = 0;
  loop = 0;
  m_is_falling_counter = 0;
  is_air_counter = 0;
  total_mass = m_robot->totalMass();
  ref_zmp_aux = hrp::Vector3::Zero();
  m_actContactStates.data.length(m_contactStates.data.length());
  for (size_t i = 0; i < m_contactStates.data.length(); i++) {
    ref_contact_states.push_back(true);
    prev_ref_contact_states.push_back(true);
    m_actContactStates.data[i] = false;
    act_contact_states.push_back(false);
    toeheel_ratio.push_back(1.0);
  }
  m_COPInfo.data.length(m_contactStates.data.length()*3); // nx, ny, fz for each end-effectors
  for (size_t i = 0; i < m_COPInfo.data.length(); i++) {
      m_COPInfo.data[i] = 0.0;
  }
  transition_time = 2.0;
  foot_origin_offset[0] = hrp::Vector3::Zero();
  foot_origin_offset[1] = hrp::Vector3::Zero();

  //
  act_cogvel_filter = boost::shared_ptr<FirstOrderLowPassFilter<hrp::Vector3> >(new FirstOrderLowPassFilter<hrp::Vector3>(4.0, dt, hrp::Vector3::Zero())); // [Hz]

  // for debug output
  m_originRefZmp.data.x = m_originRefZmp.data.y = m_originRefZmp.data.z = 0.0;
  m_originRefCog.data.x = m_originRefCog.data.y = m_originRefCog.data.z = 0.0;
  m_originRefCogVel.data.x = m_originRefCogVel.data.y = m_originRefCogVel.data.z = 0.0;
  m_originNewZmp.data.x = m_originNewZmp.data.y = m_originNewZmp.data.z = 0.0;
  m_originActZmp.data.x = m_originActZmp.data.y = m_originActZmp.data.z = 0.0;
  m_originActCog.data.x = m_originActCog.data.y = m_originActCog.data.z = 0.0;
  m_originActCogVel.data.x = m_originActCogVel.data.y = m_originActCogVel.data.z = 0.0;
  m_allRefWrench.data.length(stikp.size() * 6); // 6 is wrench dim
  m_allEEComp.data.length(stikp.size() * 6); // 6 is pos+rot dim
  m_debugData.data.length(1); m_debugData.data[0] = 0.0;

  //
  szd = new SimpleZMPDistributor(dt);
  std::vector<std::vector<Eigen::Vector2d> > support_polygon_vec;
  for (size_t i = 0; i < stikp.size(); i++) {
      support_polygon_vec.push_back(std::vector<Eigen::Vector2d>(1,Eigen::Vector2d::Zero()));
  }
  szd->set_vertices(support_polygon_vec);

  rel_ee_pos.reserve(stikp.size());
  rel_ee_rot.reserve(stikp.size());
  rel_ee_name.reserve(stikp.size());

  hrp::Sensor* sen = m_robot->sensor<hrp::RateGyroSensor>("gyrometer");
  if (sen == NULL) {
      std::cerr << "[" << m_profile.instance_name << "] WARNING! This robot model has no GyroSensor named 'gyrometer'! " << std::endl;
  }
  return RTC::RTC_OK;
}


RTC::ReturnCode_t Stabilizer::onFinalize()
{
  if (szd == NULL) {
      delete szd;
      szd = NULL;
  }
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t Stabilizer::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t Stabilizer::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

RTC::ReturnCode_t Stabilizer::onActivated(RTC::UniqueId ec_id)
{
  std::cerr << "[" << m_profile.instance_name<< "] onActivated(" << ec_id << ")" << std::endl;
  return RTC::RTC_OK;
}

RTC::ReturnCode_t Stabilizer::onDeactivated(RTC::UniqueId ec_id)
{
  Guard guard(m_mutex);
  std::cerr << "[" << m_profile.instance_name<< "] onDeactivated(" << ec_id << ")" << std::endl;
  if ( (control_mode == MODE_ST || control_mode == MODE_AIR) ) {
    sync_2_idle ();
    control_mode = MODE_IDLE;
    transition_count = 1; // sync in one controller loop
  }
  return RTC::RTC_OK;
}

#define DEBUGP ((m_debugLevel==1 && loop%200==0) || m_debugLevel > 1 )
#define DEBUGP2 (loop%10==0)
RTC::ReturnCode_t Stabilizer::onExecute(RTC::UniqueId ec_id)
{
  loop++;
  // std::cout << m_profile.instance_name<< ": onExecute(" << ec_id << ")" << std::endl;

  if (m_qRefIn.isNew()) {
    m_qRefIn.read();
  }
  if (m_qCurrentIn.isNew()) {
    m_qCurrentIn.read();
  }
  if (m_rpyIn.isNew()) {
    m_rpyIn.read();
  }
  if (m_zmpRefIn.isNew()) {
    m_zmpRefIn.read();
  }
  if (m_basePosIn.isNew()){
    m_basePosIn.read();
  }
  if (m_baseRpyIn.isNew()){
    m_baseRpyIn.read();
  }
  if (m_contactStatesIn.isNew()){
    m_contactStatesIn.read();
    for (size_t i = 0; i < m_contactStates.data.length(); i++) {
      ref_contact_states[i] = m_contactStates.data[i];
    }
  }
  if (m_toeheelRatioIn.isNew()){
    m_toeheelRatioIn.read();
    for (size_t i = 0; i < m_toeheelRatio.data.length(); i++) {
      toeheel_ratio[i] = m_toeheelRatio.data[i];
    }
  }
  if (m_controlSwingSupportTimeIn.isNew()){
    m_controlSwingSupportTimeIn.read();
  }
  for (size_t i = 0; i < m_wrenchesIn.size(); ++i) {
    if ( m_wrenchesIn[i]->isNew() ) {
      m_wrenchesIn[i]->read();
    }
  }
  for (size_t i = 0; i < m_ref_wrenchesIn.size(); ++i) {
    if ( m_ref_wrenchesIn[i]->isNew() ) {
      m_ref_wrenchesIn[i]->read();
    }
  }
  Guard guard(m_mutex);
  for (size_t i = 0; i < m_limbCOPOffsetIn.size(); ++i) {
    if ( m_limbCOPOffsetIn[i]->isNew() ) {
      m_limbCOPOffsetIn[i]->read();
      //stikp[i].localCOPPos = stikp[i].localp + stikp[i].localR * hrp::Vector3(m_limbCOPOffset[i].data.x, m_limbCOPOffset[i].data.y, m_limbCOPOffset[i].data.z);
      stikp[i].localCOPPos = stikp[i].localp + stikp[i].localR * hrp::Vector3(m_limbCOPOffset[i].data.x, 0, m_limbCOPOffset[i].data.z);
    }
  }
  if (m_qRefSeqIn.isNew()) {
    m_qRefSeqIn.read();
    is_seq_interpolating = true;
  } else {
    is_seq_interpolating = false;
  }
  if (m_walkingStatesIn.isNew()){
    m_walkingStatesIn.read();
    is_walking = m_walkingStates.data;
  }
  if (m_sbpCogOffsetIn.isNew()){
    m_sbpCogOffsetIn.read();
    sbp_cog_offset(0) = m_sbpCogOffset.data.x;
    sbp_cog_offset(1) = m_sbpCogOffset.data.y;
    sbp_cog_offset(2) = m_sbpCogOffset.data.z;
  }

  if (is_legged_robot) {
    getCurrentParameters();
    getTargetParameters();
    getActualParameters();
    calcStateForEmergencySignal();
    switch (control_mode) {
    case MODE_IDLE:
      break;
    case MODE_AIR:
      if ( transition_count == 0 && on_ground ) sync_2_st();
      break;
    case MODE_ST:
      if (st_algorithm != OpenHRP::StabilizerService::TPCC) {
        calcEEForceMomentControl();
      } else {
        calcTPCC();
      }
      if ( transition_count == 0 && !on_ground ) {
          if (is_air_counter < detection_count_to_air) ++is_air_counter;
          else control_mode = MODE_SYNC_TO_AIR;
      } else is_air_counter = 0;
      break;
    case MODE_SYNC_TO_IDLE:
      sync_2_idle();
      control_mode = MODE_IDLE;
      break;
    case MODE_SYNC_TO_AIR:
      sync_2_idle();
      control_mode = MODE_AIR;
      break;
    }
  }
  if ( m_robot->numJoints() == m_qRef.data.length() ) {
    if (is_legged_robot) {
      for ( int i = 0; i < m_robot->numJoints(); i++ ){
        m_qRef.data[i] = m_robot->joint(i)->q;
        //m_tau.data[i] = m_robot->joint(i)->u;
      }
      m_zmp.data.x = rel_act_zmp(0);
      m_zmp.data.y = rel_act_zmp(1);
      m_zmp.data.z = rel_act_zmp(2);
      m_zmp.tm = m_qRef.tm;
      m_zmpOut.write();
      m_refCP.data.x = rel_ref_cp(0);
      m_refCP.data.y = rel_ref_cp(1);
      m_refCP.data.z = rel_ref_cp(2);
      m_refCP.tm = m_qRef.tm;
      m_refCPOut.write();
      m_actCP.data.x = rel_act_cp(0);
      m_actCP.data.y = rel_act_cp(1);
      m_actCP.data.z = rel_act_cp(2);
      m_actCP.tm = m_qRef.tm;
      m_actCPOut.write();
      {
        hrp::Vector3 tmp_diff_cp = ref_foot_origin_rot * (ref_cp - act_cp - cp_offset);
        m_diffCP.data.x = tmp_diff_cp(0);
        m_diffCP.data.y = tmp_diff_cp(1);
        m_diffCP.data.z = tmp_diff_cp(2);
        m_diffCP.tm = m_qRef.tm;
        m_diffCPOut.write();
      }
      m_diffFootOriginExtMoment.data.x = diff_foot_origin_ext_moment(0);
      m_diffFootOriginExtMoment.data.y = diff_foot_origin_ext_moment(1);
      m_diffFootOriginExtMoment.data.z = diff_foot_origin_ext_moment(2);
      m_diffFootOriginExtMoment.tm = m_qRef.tm;
      m_diffFootOriginExtMomentOut.write();
      m_actContactStates.tm = m_qRef.tm;
      m_actContactStatesOut.write();
      m_COPInfo.tm = m_qRef.tm;
      m_COPInfoOut.write();
      //m_tauOut.write();
      // for debug output
      m_originRefZmp.data.x = ref_zmp(0); m_originRefZmp.data.y = ref_zmp(1); m_originRefZmp.data.z = ref_zmp(2);
      m_originRefCog.data.x = ref_cog(0); m_originRefCog.data.y = ref_cog(1); m_originRefCog.data.z = ref_cog(2);
      m_originRefCogVel.data.x = ref_cogvel(0); m_originRefCogVel.data.y = ref_cogvel(1); m_originRefCogVel.data.z = ref_cogvel(2);
      m_originNewZmp.data.x = new_refzmp(0); m_originNewZmp.data.y = new_refzmp(1); m_originNewZmp.data.z = new_refzmp(2);
      m_originActZmp.data.x = act_zmp(0); m_originActZmp.data.y = act_zmp(1); m_originActZmp.data.z = act_zmp(2);
      m_originActCog.data.x = act_cog(0); m_originActCog.data.y = act_cog(1); m_originActCog.data.z = act_cog(2);
      m_originActCogVel.data.x = act_cogvel(0); m_originActCogVel.data.y = act_cogvel(1); m_originActCogVel.data.z = act_cogvel(2);
      m_originRefZmp.tm = m_qRef.tm;
      m_originRefZmpOut.write();
      m_originRefCog.tm = m_qRef.tm;
      m_originRefCogOut.write();
      m_originRefCogVel.tm = m_qRef.tm;
      m_originRefCogVelOut.write();
      m_originNewZmp.tm = m_qRef.tm;
      m_originNewZmpOut.write();
      m_originActZmp.tm = m_qRef.tm;
      m_originActZmpOut.write();
      m_originActCog.tm = m_qRef.tm;
      m_originActCogOut.write();
      m_originActCogVel.tm = m_qRef.tm;
      m_originActCogVelOut.write();
      for (size_t i = 0; i < stikp.size(); i++) {
          for (size_t j = 0; j < 3; j++) {
              m_allRefWrench.data[6*i+j] = stikp[i].ref_force(j);
              m_allRefWrench.data[6*i+j+3] = stikp[i].ref_moment(j);
              m_allEEComp.data[6*i+j] = stikp[i].d_foot_pos(j);
              m_allEEComp.data[6*i+j+3] = stikp[i].d_foot_rpy(j);
          }
      }
      m_allRefWrench.tm = m_qRef.tm;
      m_allRefWrenchOut.write();
      m_allEEComp.tm = m_qRef.tm;
      m_allEECompOut.write();
      m_actBaseRpy.data.r = act_base_rpy(0);
      m_actBaseRpy.data.p = act_base_rpy(1);
      m_actBaseRpy.data.y = act_base_rpy(2);
      m_actBaseRpy.tm = m_qRef.tm;
      m_currentBaseRpy.data.r = current_base_rpy(0);
      m_currentBaseRpy.data.p = current_base_rpy(1);
      m_currentBaseRpy.data.y = current_base_rpy(2);
      m_currentBaseRpy.tm = m_qRef.tm;
      m_currentBasePos.data.x = current_base_pos(0);
      m_currentBasePos.data.y = current_base_pos(1);
      m_currentBasePos.data.z = current_base_pos(2);
      m_currentBasePos.tm = m_qRef.tm;
      m_actBaseRpyOut.write();
      m_currentBaseRpyOut.write();
      m_currentBasePosOut.write();
      m_debugData.tm = m_qRef.tm;
      m_debugDataOut.write();
    }
    m_qRefOut.write();
    // emergencySignal
    if (reset_emergency_flag) {
      m_emergencySignal.data = 0;
      m_emergencySignalOut.write();
      reset_emergency_flag = false;
    } else if (is_emergency) {
      m_emergencySignal.data = 1;
      m_emergencySignalOut.write();
    }
  }

  return RTC::RTC_OK;
}

void Stabilizer::getCurrentParameters ()
{
  current_root_p = m_robot->rootLink()->p;
  current_root_R = m_robot->rootLink()->R;
  for ( int i = 0; i < m_robot->numJoints(); i++ ){
    qorg[i] = m_robot->joint(i)->q;
  }
}

void Stabilizer::calcFootOriginCoords (hrp::Vector3& foot_origin_pos, hrp::Matrix33& foot_origin_rot)
{
  rats::coordinates leg_c[2], tmpc;
  hrp::Vector3 ez = hrp::Vector3::UnitZ();
  hrp::Vector3 ex = hrp::Vector3::UnitX();
  for (size_t i = 0; i < stikp.size(); i++) {
    if (stikp[i].ee_name.find("leg") == std::string::npos) continue;
    hrp::Link* target = m_robot->sensor<hrp::ForceSensor>(stikp[i].sensor_name)->link;
    leg_c[i].pos = target->p + target->R * foot_origin_offset[i];
    hrp::Vector3 xv1(target->R * ex);
    xv1(2)=0.0;
    xv1.normalize();
    hrp::Vector3 yv1(ez.cross(xv1));
    leg_c[i].rot(0,0) = xv1(0); leg_c[i].rot(1,0) = xv1(1); leg_c[i].rot(2,0) = xv1(2);
    leg_c[i].rot(0,1) = yv1(0); leg_c[i].rot(1,1) = yv1(1); leg_c[i].rot(2,1) = yv1(2);
    leg_c[i].rot(0,2) = ez(0); leg_c[i].rot(1,2) = ez(1); leg_c[i].rot(2,2) = ez(2);
  }
  if (ref_contact_states[contact_states_index_map["rleg"]] &&
      ref_contact_states[contact_states_index_map["lleg"]]) {
    rats::mid_coords(tmpc, 0.5, leg_c[0], leg_c[1]);
    foot_origin_pos = tmpc.pos;
    foot_origin_rot = tmpc.rot;
  } else if (ref_contact_states[contact_states_index_map["rleg"]]) {
    foot_origin_pos = leg_c[contact_states_index_map["rleg"]].pos;
    foot_origin_rot = leg_c[contact_states_index_map["rleg"]].rot;
  } else {
    foot_origin_pos = leg_c[contact_states_index_map["lleg"]].pos;
    foot_origin_rot = leg_c[contact_states_index_map["lleg"]].rot;
  }
}

void Stabilizer::getActualParameters ()
{
  // Actual world frame =>
  hrp::Vector3 foot_origin_pos;
  hrp::Matrix33 foot_origin_rot;
  if (st_algorithm != OpenHRP::StabilizerService::TPCC) {
    // update by current joint angles
    for ( int i = 0; i < m_robot->numJoints(); i++ ){
      m_robot->joint(i)->q = m_qCurrent.data[i];
    }
    // tempolary
    m_robot->rootLink()->p = hrp::Vector3::Zero();
    m_robot->calcForwardKinematics();
    hrp::Sensor* sen = m_robot->sensor<hrp::RateGyroSensor>("gyrometer");
    hrp::Matrix33 senR = sen->link->R * sen->localR;
    hrp::Matrix33 act_Rs(hrp::rotFromRpy(m_rpy.data.r, m_rpy.data.p, m_rpy.data.y));
    //hrp::Matrix33 act_Rs(hrp::rotFromRpy(m_rpy.data.r*0.5, m_rpy.data.p*0.5, m_rpy.data.y*0.5));
    m_robot->rootLink()->R = act_Rs * (senR.transpose() * m_robot->rootLink()->R);
    m_robot->calcForwardKinematics();
    act_base_rpy = hrp::rpyFromRot(m_robot->rootLink()->R);
    calcFootOriginCoords (foot_origin_pos, foot_origin_rot);
  } else {
    for ( int i = 0; i < m_robot->numJoints(); i++ ) {
      m_robot->joint(i)->q = qorg[i];
    }
    m_robot->rootLink()->p = current_root_p;
    m_robot->rootLink()->R = current_root_R;
    m_robot->calcForwardKinematics();
  }
  // cog
  act_cog = m_robot->calcCM();
  // zmp
  on_ground = false;
  if (st_algorithm != OpenHRP::StabilizerService::TPCC) {
    on_ground = calcZMP(act_zmp, zmp_origin_off+foot_origin_pos(2));
  } else {
    on_ground = calcZMP(act_zmp, ref_zmp(2));
  }
  // set actual contact states
  for (size_t i = 0; i < stikp.size(); i++) {
      std::string limb_name = stikp[i].ee_name;
      size_t idx = contact_states_index_map[limb_name];
      act_contact_states[idx] = isContact(idx);
      m_actContactStates.data[idx] = act_contact_states[idx];
  }
  // <= Actual world frame

  // convert absolute (in st) -> root-link relative
  rel_act_zmp = m_robot->rootLink()->R.transpose() * (act_zmp - m_robot->rootLink()->p);
  if (st_algorithm != OpenHRP::StabilizerService::TPCC) {
    // Actual foot_origin frame =>
    act_zmp = foot_origin_rot.transpose() * (act_zmp - foot_origin_pos);
    act_cog = foot_origin_rot.transpose() * (act_cog - foot_origin_pos);
    //act_cogvel = foot_origin_rot.transpose() * act_cogvel;
    if (ref_contact_states != prev_ref_contact_states) {
      act_cogvel = (foot_origin_rot.transpose() * prev_act_foot_origin_rot) * act_cogvel;
    } else {
      act_cogvel = (act_cog - prev_act_cog)/dt;
    }
    act_cogvel = act_cogvel_filter->passFilter(act_cogvel);
    prev_act_cog = act_cog;
    //act_root_rot = m_robot->rootLink()->R;
    for (size_t i = 0; i < stikp.size(); i++) {
      hrp::Link* target = m_robot->link(stikp[i].target_name);
      //hrp::Vector3 act_ee_p = target->p + target->R * stikp[i].localCOPPos;
      hrp::Vector3 _act_ee_p = target->p + target->R * stikp[i].localp;
      act_ee_p[i] = foot_origin_rot.transpose() * (_act_ee_p - foot_origin_pos);
      act_ee_R[i] = foot_origin_rot.transpose() * (target->R * stikp[i].localR);
    }
    // capture point
    act_cp = act_cog + act_cogvel / std::sqrt(eefm_gravitational_acceleration / (act_cog - act_zmp)(2));
    rel_act_cp = hrp::Vector3(act_cp(0), act_cp(1), act_zmp(2));
    rel_act_cp = m_robot->rootLink()->R.transpose() * ((foot_origin_pos + foot_origin_rot * rel_act_cp) - m_robot->rootLink()->p);
    // <= Actual foot_origin frame

    // Actual world frame =>
    // new ZMP calculation
    // Kajita's feedback law
    //   Basically Equation (26) in the paper [1].
    hrp::Vector3 dcog=foot_origin_rot * (ref_cog - act_cog);
    hrp::Vector3 dcogvel=foot_origin_rot * (ref_cogvel - act_cogvel);
    hrp::Vector3 dzmp=foot_origin_rot * (ref_zmp - act_zmp);
    new_refzmp = foot_origin_rot * new_refzmp + foot_origin_pos;
    for (size_t i = 0; i < 2; i++) {
      new_refzmp(i) += eefm_k1[i] * transition_smooth_gain * dcog(i) + eefm_k2[i] * transition_smooth_gain * dcogvel(i) + eefm_k3[i] * transition_smooth_gain * dzmp(i) + ref_zmp_aux(i);
    }
    if (DEBUGP) {
      // All state variables are foot_origin coords relative
      std::cerr << "[" << m_profile.instance_name << "] state values" << std::endl;
      std::cerr << "[" << m_profile.instance_name << "]   "
                << "ref_cog    = " << hrp::Vector3(ref_cog*1e3).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]"))
                << ", act_cog    = " << hrp::Vector3(act_cog*1e3).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[mm]" << std::endl;
      std::cerr << "[" << m_profile.instance_name << "]   "
                << "ref_cogvel = " << hrp::Vector3(ref_cogvel*1e3).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]"))
                << ", act_cogvel = " << hrp::Vector3(act_cogvel*1e3).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[mm/s]" << std::endl;
      std::cerr << "[" << m_profile.instance_name << "]   "
                << "ref_zmp    = " << hrp::Vector3(ref_zmp*1e3).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]"))
                << ", act_zmp    = " << hrp::Vector3(act_zmp*1e3).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[mm]" << std::endl;
      hrp::Vector3 tmpnew_refzmp;
      tmpnew_refzmp = foot_origin_rot.transpose()*(new_refzmp-foot_origin_pos); // Actual world -> foot origin relative
      std::cerr << "[" << m_profile.instance_name << "]   "
                << "new_zmp    = " << hrp::Vector3(tmpnew_refzmp*1e3).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]"))
                << ", dif_zmp    = " << hrp::Vector3((tmpnew_refzmp-ref_zmp)*1e3).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[mm]" << std::endl;
    }

    std::vector<std::string> ee_name;
    // distribute new ZMP into foot force & moment
    std::vector<hrp::Vector3> tmp_ref_force, tmp_ref_moment;
    std::vector<double> limb_gains;
    std::vector<hrp::dvector6> ee_forcemoment_distribution_weight;
    std::vector<double> tmp_toeheel_ratio;
    if (control_mode == MODE_ST) {
      std::vector<hrp::Vector3> ee_pos, cop_pos;
      std::vector<hrp::Matrix33> ee_rot;
      std::vector<bool> is_contact_list;
      is_contact_list.reserve(stikp.size());
      for (size_t i = 0; i < stikp.size(); i++) {
          STIKParam& ikp = stikp[i];
          if (!is_feedback_control_enable[i]) continue;
          hrp::Link* target = m_robot->link(ikp.target_name);
          ee_pos.push_back(target->p + target->R * ikp.localp);
          cop_pos.push_back(target->p + target->R * ikp.localCOPPos);
          ee_rot.push_back(target->R * ikp.localR);
          ee_name.push_back(ikp.ee_name);
          limb_gains.push_back(ikp.swing_support_gain);
          tmp_ref_force.push_back(hrp::Vector3(foot_origin_rot * ref_force[i]));
          tmp_ref_moment.push_back(hrp::Vector3(foot_origin_rot * ref_moment[i]));
          rel_ee_pos.push_back(foot_origin_rot.transpose() * (ee_pos.back() - foot_origin_pos));
          rel_ee_rot.push_back(foot_origin_rot.transpose() * ee_rot.back());
          rel_ee_name.push_back(ee_name.back());
          is_contact_list.push_back(act_contact_states[i]);
          // std::cerr << ee_forcemoment_distribution_weight[i] << std::endl;
          ee_forcemoment_distribution_weight.push_back(hrp::dvector6::Zero(6,1));
          for (size_t j = 0; j < 6; j++) {
              ee_forcemoment_distribution_weight[i][j] = ikp.eefm_ee_forcemoment_distribution_weight[j];
          }
          tmp_toeheel_ratio.push_back(toeheel_ratio[i]);
      }

      // All state variables are foot_origin coords relative
      if (DEBUGP) {
          std::cerr << "[" << m_profile.instance_name << "] ee values" << std::endl;
          hrp::Vector3 tmpp;
          for (size_t i = 0; i < ee_name.size(); i++) {
              tmpp = foot_origin_rot.transpose()*(ee_pos[i]-foot_origin_pos);
              std::cerr << "[" << m_profile.instance_name << "]   "
                        << "ee_pos (" << ee_name[i] << ")    = " << hrp::Vector3(tmpp*1e3).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]"));
              tmpp = foot_origin_rot.transpose()*(cop_pos[i]-foot_origin_pos);
              std::cerr << ", cop_pos (" << ee_name[i] << ")    = " << hrp::Vector3(tmpp*1e3).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[mm]" << std::endl;
          }
      }

      // truncate ZMP
      if (use_zmp_truncation) {
        Eigen::Vector2d tmp_new_refzmp(new_refzmp.head(2));
        szd->get_vertices(support_polygon_vetices);
        szd->calc_convex_hull(support_polygon_vetices, ref_contact_states, ee_pos, ee_rot);
        if (!szd->is_inside_support_polygon(tmp_new_refzmp, hrp::Vector3::Zero(), true, std::string(m_profile.instance_name))) new_refzmp.head(2) = tmp_new_refzmp;
      }

      // Distribute ZMP into each EE force/moment at each COP
      if (st_algorithm == OpenHRP::StabilizerService::EEFM) {
          // Modified version of distribution in Equation (4)-(6) and (10)-(13) in the paper [1].
          szd->distributeZMPToForceMoments(tmp_ref_force, tmp_ref_moment,
                                           ee_pos, cop_pos, ee_rot, ee_name, limb_gains, tmp_toeheel_ratio,
                                           new_refzmp, hrp::Vector3(foot_origin_rot * ref_zmp + foot_origin_pos),
                                           eefm_gravitational_acceleration * total_mass, dt,
                                           DEBUGP, std::string(m_profile.instance_name));
      } else if (st_algorithm == OpenHRP::StabilizerService::EEFMQP) {
          szd->distributeZMPToForceMomentsQP(tmp_ref_force, tmp_ref_moment,
                                             ee_pos, cop_pos, ee_rot, ee_name, limb_gains, tmp_toeheel_ratio,
                                             new_refzmp, hrp::Vector3(foot_origin_rot * ref_zmp + foot_origin_pos),
                                             eefm_gravitational_acceleration * total_mass, dt,
                                             DEBUGP, std::string(m_profile.instance_name),
                                             (st_algorithm == OpenHRP::StabilizerService::EEFMQPCOP));
      } else if (st_algorithm == OpenHRP::StabilizerService::EEFMQPCOP) {
          szd->distributeZMPToForceMomentsPseudoInverse(tmp_ref_force, tmp_ref_moment,
                                             ee_pos, cop_pos, ee_rot, ee_name, limb_gains, tmp_toeheel_ratio,
                                             new_refzmp, hrp::Vector3(foot_origin_rot * ref_zmp + foot_origin_pos),
                                             eefm_gravitational_acceleration * total_mass, dt,
                                             DEBUGP, std::string(m_profile.instance_name),
                                             (st_algorithm == OpenHRP::StabilizerService::EEFMQPCOP), is_contact_list);
      } else if (st_algorithm == OpenHRP::StabilizerService::EEFMQPCOP2) {
          szd->distributeZMPToForceMomentsPseudoInverse2(tmp_ref_force, tmp_ref_moment,
                                                         ee_pos, cop_pos, ee_rot, ee_name, limb_gains, tmp_toeheel_ratio,
                                                         new_refzmp, hrp::Vector3(foot_origin_rot * ref_zmp + foot_origin_pos),
                                                         foot_origin_rot * ref_total_force, foot_origin_rot * ref_total_moment,
                                                         ee_forcemoment_distribution_weight,
                                                         eefm_gravitational_acceleration * total_mass, dt,
                                                         DEBUGP, std::string(m_profile.instance_name));
      }
      // for debug output
      new_refzmp = foot_origin_rot.transpose() * (new_refzmp - foot_origin_pos);
    }

    // foor modif
    if (control_mode == MODE_ST) {
      hrp::Vector3 f_diff(hrp::Vector3::Zero());
      std::vector<bool> large_swing_f_diff(3, false);
      // moment control
      act_total_foot_origin_moment = hrp::Vector3::Zero();
      for (size_t i = 0; i < stikp.size(); i++) {
        STIKParam& ikp = stikp[i];
        std::vector<bool> large_swing_m_diff(3, false);
        if (!is_feedback_control_enable[i]) continue;
        hrp::Sensor* sensor = m_robot->sensor<hrp::ForceSensor>(ikp.sensor_name);
        hrp::Link* target = m_robot->link(ikp.target_name);
        // Convert moment at COP => moment at ee
        size_t idx = contact_states_index_map[ikp.ee_name];
        ikp.ref_moment = tmp_ref_moment[idx] + ((target->R * ikp.localCOPPos + target->p) - (target->R * ikp.localp + target->p)).cross(tmp_ref_force[idx]);
        ikp.ref_force = tmp_ref_force[idx];
        // Actual world frame =>
        hrp::Vector3 sensor_force = (sensor->link->R * sensor->localR) * hrp::Vector3(m_wrenches[i].data[0], m_wrenches[i].data[1], m_wrenches[i].data[2]);
        hrp::Vector3 sensor_moment = (sensor->link->R * sensor->localR) * hrp::Vector3(m_wrenches[i].data[3], m_wrenches[i].data[4], m_wrenches[i].data[5]);
        //hrp::Vector3 ee_moment = ((sensor->link->R * sensor->localPos + sensor->link->p) - (target->R * ikp.localCOPPos + target->p)).cross(sensor_force) + sensor_moment;
        hrp::Vector3 ee_moment = ((sensor->link->R * sensor->localPos + sensor->link->p) - (target->R * ikp.localp + target->p)).cross(sensor_force) + sensor_moment;
        // <= Actual world frame
        hrp::Matrix33 ee_R(target->R * ikp.localR);
        // Actual ee frame =>
        ikp.ref_moment = ee_R.transpose() * ikp.ref_moment;
        ikp.ref_force = ee_R.transpose() * ikp.ref_force;
        sensor_force = ee_R.transpose() * sensor_force;
        ee_moment = ee_R.transpose() * ee_moment;
        if ( i == 0 ) f_diff += -1*sensor_force;
        else f_diff += sensor_force;
        for (size_t j = 0; j < 3; ++j) {
            if ((!ref_contact_states[i] || !act_contact_states[i]) && fabs(ikp.ref_force(j) - sensor_force(j)) > eefm_swing_damping_force_thre[j]) large_swing_f_diff[j] = true;
            if ((!ref_contact_states[i] || !act_contact_states[i]) && (fabs(ikp.ref_moment(j) - ee_moment(j)) > eefm_swing_damping_moment_thre[j])) large_swing_m_diff[j] = true;
        }
        // Moment limitation
        ikp.ref_moment = ee_R * vlimit((ee_R.transpose() * ikp.ref_moment), ikp.eefm_ee_moment_limit);
        // calcDampingControl
        // ee_d_foot_rpy and ee_d_foot_pos is (actual) end effector coords relative value because these use end effector coords relative force & moment
        { // Rot
          //   Basically Equation (16) and (17) in the paper [1]
          hrp::Vector3 tmp_damping_gain;
          for (size_t j = 0; j < 3; ++j) {
              if (!eefm_use_swing_damping || !large_swing_m_diff[j]) tmp_damping_gain(j) = (1-transition_smooth_gain) * ikp.eefm_rot_damping_gain(j) * 10 + transition_smooth_gain * ikp.eefm_rot_damping_gain(j);
              else tmp_damping_gain(j) = (1-transition_smooth_gain) * eefm_swing_rot_damping_gain(j) * 10 + transition_smooth_gain * eefm_swing_rot_damping_gain(j);
          }
          ikp.ee_d_foot_rpy = calcDampingControl(ikp.ref_moment, ee_moment, ikp.ee_d_foot_rpy, tmp_damping_gain, ikp.eefm_rot_time_const);
          ikp.ee_d_foot_rpy = vlimit(ikp.ee_d_foot_rpy, -1 * ikp.eefm_rot_compensation_limit, ikp.eefm_rot_compensation_limit);
        }
        if (!eefm_use_force_difference_control) { // Pos
            hrp::Vector3 tmp_damping_gain = (1-transition_smooth_gain) * ikp.eefm_pos_damping_gain * 10 + transition_smooth_gain * ikp.eefm_pos_damping_gain;
            ikp.ee_d_foot_pos = calcDampingControl(ikp.ref_force, sensor_force, ikp.ee_d_foot_pos, tmp_damping_gain, ikp.eefm_pos_time_const_support);
            ikp.ee_d_foot_pos = vlimit(ikp.ee_d_foot_pos, -1 * ikp.eefm_pos_compensation_limit, ikp.eefm_pos_compensation_limit);
        }
        // Convert force & moment as foot origin coords relative
        ikp.ref_moment = foot_origin_rot.transpose() * ee_R * ikp.ref_moment;
        ikp.ref_force = foot_origin_rot.transpose() * ee_R * ikp.ref_force;
        sensor_force = foot_origin_rot.transpose() * ee_R * sensor_force;
        ee_moment = foot_origin_rot.transpose() * ee_R * ee_moment;
        ikp.d_foot_rpy = foot_origin_rot.transpose() * ee_R * ikp.ee_d_foot_rpy;
        ikp.d_foot_pos = foot_origin_rot.transpose() * ee_R * ikp.ee_d_foot_pos;
        // tilt Check : only flat plane is supported
        {
            hrp::Vector3 plane_x = target_ee_R[i].col(0);
            hrp::Vector3 plane_y = target_ee_R[i].col(1);
            hrp::Matrix33 act_ee_R_world = target->R * stikp[i].localR;
            hrp::Vector3 normal_vector = act_ee_R_world.col(2);
            /* projected_normal = c1 * plane_x + c2 * plane_y : c1 = plane_x.dot(normal_vector), c2 = plane_y.dot(normal_vector) because (normal-vector - projected_normal) is orthogonal to plane */
            projected_normal.at(i) = plane_x.dot(normal_vector) * plane_x + plane_y.dot(normal_vector) * plane_y;
            act_force.at(i) = sensor_force;
        }
        //act_total_foot_origin_moment += (target->R * ikp.localCOPPos + target->p).cross(sensor_force) + ee_moment;
        act_total_foot_origin_moment += (target->R * ikp.localp + target->p - foot_origin_pos).cross(sensor_force) + ee_moment;
      }
      act_total_foot_origin_moment = foot_origin_rot.transpose() * act_total_foot_origin_moment;

      if (eefm_use_force_difference_control) {
          // fxyz control
          // foot force difference control version
          //   Basically Equation (18) in the paper [1]
          hrp::Vector3 ref_f_diff = (stikp[1].ref_force-stikp[0].ref_force);
          if (ref_contact_states != prev_ref_contact_states) pos_ctrl = (foot_origin_rot.transpose() * prev_act_foot_origin_rot) * pos_ctrl;
          if (eefm_use_swing_damping) {
            hrp::Vector3 tmp_damping_gain;
            for (size_t i = 0; i < 3; ++i) {
                if (!large_swing_f_diff[i]) tmp_damping_gain(i) = (1-transition_smooth_gain) * stikp[0].eefm_pos_damping_gain(i) * 10 + transition_smooth_gain * stikp[0].eefm_pos_damping_gain(i);
                else tmp_damping_gain(i) = (1-transition_smooth_gain) * eefm_swing_pos_damping_gain(i) * 10 + transition_smooth_gain * eefm_swing_pos_damping_gain(i);
            }
            pos_ctrl = calcDampingControl (ref_f_diff, f_diff, pos_ctrl,
                                           tmp_damping_gain, stikp[0].eefm_pos_time_const_support);
          } else {
            if ( (ref_contact_states[contact_states_index_map["rleg"]] && ref_contact_states[contact_states_index_map["lleg"]]) // Reference : double support phase
                 || (act_contact_states[0] && act_contact_states[1]) ) { // Actual : double support phase
              // Temporarily use first pos damping gain (stikp[0])
              hrp::Vector3 tmp_damping_gain = (1-transition_smooth_gain) * stikp[0].eefm_pos_damping_gain * 10 + transition_smooth_gain * stikp[0].eefm_pos_damping_gain;
              pos_ctrl = calcDampingControl (ref_f_diff, f_diff, pos_ctrl,
                                             tmp_damping_gain, stikp[0].eefm_pos_time_const_support);
            } else {
              double remain_swing_time;
              if ( !ref_contact_states[contact_states_index_map["rleg"]] ) { // rleg swing
                  remain_swing_time = m_controlSwingSupportTime.data[contact_states_index_map["rleg"]];
              } else { // lleg swing
                  remain_swing_time = m_controlSwingSupportTime.data[contact_states_index_map["lleg"]];
              }
              // std::cerr << "st " << remain_swing_time << " rleg " << contact_states[contact_states_index_map["rleg"]] << " lleg " << contact_states[contact_states_index_map["lleg"]] << std::endl;
              double tmp_ratio = std::max(0.0, std::min(1.0, 1.0 - (remain_swing_time-eefm_pos_margin_time)/eefm_pos_transition_time)); // 0=>1
              // Temporarily use first pos damping gain (stikp[0])
              hrp::Vector3 tmp_damping_gain = (1-transition_smooth_gain) * stikp[0].eefm_pos_damping_gain * 10 + transition_smooth_gain * stikp[0].eefm_pos_damping_gain;
              hrp::Vector3 tmp_time_const = (1-tmp_ratio)*eefm_pos_time_const_swing*hrp::Vector3::Ones()+tmp_ratio*stikp[0].eefm_pos_time_const_support;
              pos_ctrl = calcDampingControl (tmp_ratio * ref_f_diff, tmp_ratio * f_diff, pos_ctrl, tmp_damping_gain, tmp_time_const);
            }
          }
          // zctrl = vlimit(zctrl, -0.02, 0.02);
          // Temporarily use first pos compensation limit (stikp[0])
          pos_ctrl = vlimit(pos_ctrl, -1 * stikp[0].eefm_pos_compensation_limit * 2, stikp[0].eefm_pos_compensation_limit * 2);
          // Divide pos_ctrl into rfoot and lfoot
          stikp[0].d_foot_pos = -0.5 * pos_ctrl;
          stikp[1].d_foot_pos = 0.5 * pos_ctrl;
      }
      if (DEBUGP) {
        std::cerr << "[" << m_profile.instance_name << "] Control values" << std::endl;
        if (eefm_use_force_difference_control) {
            std::cerr << "[" << m_profile.instance_name << "]   "
                      << "pos_ctrl    = [" << pos_ctrl(0)*1e3 << " " << pos_ctrl(1)*1e3 << " "<< pos_ctrl(2)*1e3 << "] [mm]" << std::endl;
        }
        for (size_t i = 0; i < ee_name.size(); i++) {
            std::cerr << "[" << m_profile.instance_name << "]   "
                      << "d_foot_pos (" << ee_name[i] << ")  = [" << stikp[i].d_foot_pos(0)*1e3 << " " << stikp[i].d_foot_pos(1)*1e3 << " " << stikp[i].d_foot_pos(2)*1e3 << "] [mm], "
                      << "d_foot_rpy (" << ee_name[i] << ")  = [" << stikp[i].d_foot_rpy(0)*180.0/M_PI << " " << stikp[i].d_foot_rpy(1)*180.0/M_PI << " " << stikp[i].d_foot_rpy(2)*180.0/M_PI << "] [deg]" << std::endl;
        }
      }
      // foot force independent damping control
      // for (size_t i = 0; i < 2; i++) {
      //   f_zctrl[i] = calcDampingControl (ref_force[i](2),
      //                                    fz[i], f_zctrl[i], eefm_pos_damping_gain, eefm_pos_time_const);
      //   f_zctrl[i] = vlimit(f_zctrl[i], -0.05, 0.05);
      // }
      calcDiffFootOriginExtMoment ();
    }
  } // st_algorithm == OpenHRP::StabilizerService::EEFM

  for ( int i = 0; i < m_robot->numJoints(); i++ ){
    m_robot->joint(i)->q = qrefv[i];
  }
  m_robot->rootLink()->p = target_root_p;
  m_robot->rootLink()->R = target_root_R;
  if ( !(control_mode == MODE_IDLE || control_mode == MODE_AIR) ) {
    for (size_t i = 0; i < jpe_v.size(); i++) {
      if (is_ik_enable[i]) {
        for ( int j = 0; j < jpe_v[i]->numJoints(); j++ ){
          int idx = jpe_v[i]->joint(j)->jointId;
          m_robot->joint(idx)->q = qorg[idx];
        }
      }
    }
    m_robot->rootLink()->p(0) = current_root_p(0);
    m_robot->rootLink()->p(1) = current_root_p(1);
    m_robot->rootLink()->R = current_root_R;
    m_robot->calcForwardKinematics();
  }
  copy (ref_contact_states.begin(), ref_contact_states.end(), prev_ref_contact_states.begin());
  if (control_mode != MODE_ST) d_pos_z_root = 0.0;
  prev_act_foot_origin_rot = foot_origin_rot;
}

void Stabilizer::getTargetParameters ()
{
  // Reference world frame =>
  // update internal robot model
  if ( transition_count == 0 ) {
    transition_smooth_gain = 1.0;
  } else {
    double max_transition_count = calcMaxTransitionCount();
    transition_smooth_gain = 1/(1+exp(-9.19*(((max_transition_count - std::fabs(transition_count)) / max_transition_count) - 0.5)));
  }
  if (transition_count > 0) {
    for ( int i = 0; i < m_robot->numJoints(); i++ ){
      m_robot->joint(i)->q = ( m_qRef.data[i] - transition_joint_q[i] ) * transition_smooth_gain + transition_joint_q[i];
    }
  } else {
    for ( int i = 0; i < m_robot->numJoints(); i++ ){
      m_robot->joint(i)->q = m_qRef.data[i];
    }
  }
  if ( transition_count < 0 ) {
    transition_count++;
  } else if ( transition_count > 0 ) {
    if ( transition_count == 1 ) {
      std::cerr << "[" << m_profile.instance_name << "] [" << m_qRef.tm << "] Move to MODE_IDLE" << std::endl;
      reset_emergency_flag = true;
    }
    transition_count--;
  }
  for ( int i = 0; i < m_robot->numJoints(); i++ ){
    qrefv[i] = m_robot->joint(i)->q;
  }
  m_robot->rootLink()->p = hrp::Vector3(m_basePos.data.x, m_basePos.data.y, m_basePos.data.z);
  target_root_p = m_robot->rootLink()->p;
  target_root_R = hrp::rotFromRpy(m_baseRpy.data.r, m_baseRpy.data.p, m_baseRpy.data.y);
  m_robot->rootLink()->R = target_root_R;
  m_robot->calcForwardKinematics();
  ref_zmp = m_robot->rootLink()->R * hrp::Vector3(m_zmpRef.data.x, m_zmpRef.data.y, m_zmpRef.data.z) + m_robot->rootLink()->p; // base frame -> world frame
  hrp::Vector3 foot_origin_pos;
  hrp::Matrix33 foot_origin_rot;
  calcFootOriginCoords (foot_origin_pos, foot_origin_rot);
  if (st_algorithm != OpenHRP::StabilizerService::TPCC) {
    // apply inverse system
    hrp::Vector3 tmp_ref_zmp = ref_zmp + eefm_zmp_delay_time_const[0] * (ref_zmp - prev_ref_zmp) / dt;
    prev_ref_zmp = ref_zmp;
    ref_zmp = tmp_ref_zmp;
  }
  ref_cog = m_robot->calcCM();
  ref_total_force = hrp::Vector3::Zero();
  ref_total_moment = hrp::Vector3::Zero(); // Total moment around reference ZMP tmp
  ref_total_foot_origin_moment = hrp::Vector3::Zero();
  for (size_t i = 0; i < stikp.size(); i++) {
    hrp::Link* target = m_robot->link(stikp[i].target_name);
    //target_ee_p[i] = target->p + target->R * stikp[i].localCOPPos;
    target_ee_p[i] = target->p + target->R * stikp[i].localp;
    target_ee_R[i] = target->R * stikp[i].localR;
    ref_force[i] = hrp::Vector3(m_ref_wrenches[i].data[0], m_ref_wrenches[i].data[1], m_ref_wrenches[i].data[2]);
    ref_moment[i] = hrp::Vector3(m_ref_wrenches[i].data[3], m_ref_wrenches[i].data[4], m_ref_wrenches[i].data[5]);
    ref_total_force += ref_force[i];
#ifdef FORCE_MOMENT_DIFF_CONTROL
    // Force/moment diff control
    ref_total_moment += (target_ee_p[i]-ref_zmp).cross(ref_force[i]);
#else
    // Force/moment control
    ref_total_moment += (target_ee_p[i]-ref_zmp).cross(hrp::Vector3(m_ref_wrenches[i].data[0], m_ref_wrenches[i].data[1], m_ref_wrenches[i].data[2]))
        + hrp::Vector3(m_ref_wrenches[i].data[3], m_ref_wrenches[i].data[4], m_ref_wrenches[i].data[5]);
#endif
    if (is_feedback_control_enable[i]) {
        ref_total_foot_origin_moment += (target_ee_p[i]-foot_origin_pos).cross(ref_force[i]) + ref_moment[i];
    }
  }
  // <= Reference world frame

  // Reset prev_ref_cog for transition (MODE_IDLE=>MODE_ST) because the coordinates for ref_cog differs among st algorithms.
  if (transition_count == (-1 * calcMaxTransitionCount() + 1)) { // max transition count. In MODE_IDLE => MODE_ST, transition_count is < 0 and upcounter. "+ 1" is upcount at the beginning of this function.
      prev_ref_cog = ref_cog;
      std::cerr << "[" << m_profile.instance_name << "]   Reset prev_ref_cog for transition (MODE_IDLE=>MODE_ST)." << std::endl;
  }

  if (st_algorithm != OpenHRP::StabilizerService::TPCC) {
    // Reference foot_origin frame =>
    // initialize for new_refzmp
    new_refzmp = ref_zmp;
    rel_cog = m_robot->rootLink()->R.transpose() * (ref_cog-m_robot->rootLink()->p);
    // convert world (current-tmp) => local (foot_origin)
    zmp_origin_off = ref_zmp(2) - foot_origin_pos(2);
    ref_zmp = foot_origin_rot.transpose() * (ref_zmp - foot_origin_pos);
    ref_cog = foot_origin_rot.transpose() * (ref_cog - foot_origin_pos);
    new_refzmp = foot_origin_rot.transpose() * (new_refzmp - foot_origin_pos);
    if (ref_contact_states != prev_ref_contact_states) {
      ref_cogvel = (foot_origin_rot.transpose() * prev_ref_foot_origin_rot) * ref_cogvel;
    } else {
      ref_cogvel = (ref_cog - prev_ref_cog)/dt;
    }
    prev_ref_foot_origin_rot = ref_foot_origin_rot = foot_origin_rot;
    for (size_t i = 0; i < stikp.size(); i++) {
      stikp[i].target_ee_diff_p = foot_origin_rot.transpose() * (target_ee_p[i] - foot_origin_pos);
      stikp[i].target_ee_diff_r = foot_origin_rot.transpose() * target_ee_R[i];
      ref_force[i] = foot_origin_rot.transpose() * ref_force[i];
      ref_moment[i] = foot_origin_rot.transpose() * ref_moment[i];
    }
    ref_total_foot_origin_moment = foot_origin_rot.transpose() * ref_total_foot_origin_moment;
    ref_total_force = foot_origin_rot.transpose() * ref_total_force;
    ref_total_moment = foot_origin_rot.transpose() * ref_total_moment;
    target_foot_origin_rot = foot_origin_rot;
    // capture point
    ref_cp = ref_cog + ref_cogvel / std::sqrt(eefm_gravitational_acceleration / (ref_cog - ref_zmp)(2));
    rel_ref_cp = hrp::Vector3(ref_cp(0), ref_cp(1), ref_zmp(2));
    rel_ref_cp = m_robot->rootLink()->R.transpose() * ((foot_origin_pos + foot_origin_rot * rel_ref_cp) - m_robot->rootLink()->p);
    sbp_cog_offset = foot_origin_rot.transpose() * sbp_cog_offset;
    // <= Reference foot_origin frame
  } else {
    ref_cogvel = (ref_cog - prev_ref_cog)/dt;
  } // st_algorithm == OpenHRP::StabilizerService::EEFM
  prev_ref_cog = ref_cog;
  // Calc swing support limb gain param
  calcSwingSupportLimbGain();
}

bool Stabilizer::calcZMP(hrp::Vector3& ret_zmp, const double zmp_z)
{
  double tmpzmpx = 0;
  double tmpzmpy = 0;
  double tmpfz = 0, tmpfz2 = 0.0;
  for (size_t i = 0; i < stikp.size(); i++) {
    if (!is_zmp_calc_enable[i]) continue;
    hrp::ForceSensor* sensor = m_robot->sensor<hrp::ForceSensor>(stikp[i].sensor_name);
    hrp::Vector3 fsp = sensor->link->p + sensor->link->R * sensor->localPos;
    hrp::Matrix33 tmpR;
    rats::rotm3times(tmpR, sensor->link->R, sensor->localR);
    hrp::Vector3 nf = tmpR * hrp::Vector3(m_wrenches[i].data[0], m_wrenches[i].data[1], m_wrenches[i].data[2]);
    hrp::Vector3 nm = tmpR * hrp::Vector3(m_wrenches[i].data[3], m_wrenches[i].data[4], m_wrenches[i].data[5]);
    tmpzmpx += nf(2) * fsp(0) - (fsp(2) - zmp_z) * nf(0) - nm(1);
    tmpzmpy += nf(2) * fsp(1) - (fsp(2) - zmp_z) * nf(1) + nm(0);
    tmpfz += nf(2);
    // calc ee-local COP
    hrp::Link* target = m_robot->link(stikp[i].target_name);
    hrp::Matrix33 eeR = target->R * stikp[i].localR;
    hrp::Vector3 ee_fsp = eeR.transpose() * (fsp - (target->p + target->R * stikp[i].localp)); // ee-local force sensor pos
    nf = eeR.transpose() * nf;
    nm = eeR.transpose() * nm;
    // ee-local total moment and total force at ee position
    double tmpcopmy = nf(2) * ee_fsp(0) - nf(0) * ee_fsp(2) - nm(1);
    double tmpcopmx = nf(2) * ee_fsp(1) - nf(1) * ee_fsp(2) + nm(0);
    double tmpcopfz = nf(2);
    m_COPInfo.data[i*3] = tmpcopmx;
    m_COPInfo.data[i*3+1] = tmpcopmy;
    m_COPInfo.data[i*3+2] = tmpcopfz;
    prev_act_force_z[i] = 0.85 * prev_act_force_z[i] + 0.15 * nf(2); // filter, cut off 5[Hz]
    tmpfz2 += prev_act_force_z[i];
  }
  if (tmpfz2 < contact_decision_threshold) {
    ret_zmp = act_zmp;
    return false; // in the air
  } else {
    ret_zmp = hrp::Vector3(tmpzmpx / tmpfz, tmpzmpy / tmpfz, zmp_z);
    return true; // on ground
  }
};

void Stabilizer::calcStateForEmergencySignal()
{
  // COP Check
  bool is_cop_outside = false;
  if (DEBUGP) {
      std::cerr << "[" << m_profile.instance_name << "] Check Emergency State (seq = " << (is_seq_interpolating?"interpolating":"empty") << ")" << std::endl;
  }
  if (on_ground && transition_count == 0 && control_mode == MODE_ST) {
    if (DEBUGP) {
        std::cerr << "[" << m_profile.instance_name << "] COP check" << std::endl;
    }
    for (size_t i = 0; i < stikp.size(); i++) {
      if (stikp[i].ee_name.find("leg") == std::string::npos) continue;
      // check COP inside
      if (m_COPInfo.data[i*3+2] > 20.0 ) {
        hrp::Vector3 tmpcop(m_COPInfo.data[i*3+1]/m_COPInfo.data[i*3+2], m_COPInfo.data[i*3]/m_COPInfo.data[i*3+2], 0);
        is_cop_outside = is_cop_outside ||
            (!szd->is_inside_foot(tmpcop, stikp[i].ee_name=="lleg", cop_check_margin) ||
             szd->is_front_of_foot(tmpcop, cop_check_margin) ||
             szd->is_rear_of_foot(tmpcop, cop_check_margin));
        if (DEBUGP) {
            std::cerr << "[" << m_profile.instance_name << "]   [" << stikp[i].ee_name << "] "
                      << "outside(" << !szd->is_inside_foot(tmpcop, stikp[i].ee_name=="lleg", cop_check_margin) << ") "
                      << "front(" << szd->is_front_of_foot(tmpcop, cop_check_margin) << ") "
                      << "rear(" << szd->is_rear_of_foot(tmpcop, cop_check_margin) << ")" << std::endl;
        }
      } else {
        is_cop_outside = true;
      }
    }
  } else {
    is_cop_outside = false;
  }
  // CP Check
  bool is_cp_outside = false;
  if (on_ground && transition_count == 0 && control_mode == MODE_ST) {
    Eigen::Vector2d tmp_cp = act_cp.head(2);
    szd->get_margined_vertices(margined_support_polygon_vetices);
    szd->calc_convex_hull(margined_support_polygon_vetices, act_contact_states, rel_ee_pos, rel_ee_rot);
    if (!is_walking || is_estop_while_walking) is_cp_outside = !szd->is_inside_support_polygon(tmp_cp, - sbp_cog_offset);
    if (DEBUGP) {
      std::cerr << "[" << m_profile.instance_name << "] CP value " << "[" << act_cp(0) << "," << act_cp(1) << "] [m], "
                << "sbp cog offset [" << sbp_cog_offset(0) << " " << sbp_cog_offset(1) << "], outside ? "
                << (is_cp_outside?"Outside":"Inside")
                << std::endl;
    }
    if (is_cp_outside) {
      if (initial_cp_too_large_error || loop % static_cast <int>(0.2/dt) == 0 ) { // once per 0.2[s]
        std::cerr << "[" << m_profile.instance_name << "] [" << m_qRef.tm
                  << "] CP too large error " << "[" << act_cp(0) << "," << act_cp(1) << "] [m]" << std::endl;
      }
      initial_cp_too_large_error = false;
    } else {
      initial_cp_too_large_error = true;
    }
  }
  // tilt Check
  hrp::Vector3 fall_direction = hrp::Vector3::Zero();
  bool is_falling = false, will_fall = false;
  {
      double total_force = 0.0;
      for (size_t i = 0; i < stikp.size(); i++) {
          if (is_zmp_calc_enable[i]) {
              if (is_walking) {
                  if (projected_normal.at(i).norm() > sin(tilt_margin[0])) {
                      will_fall = true;
                      if (m_will_fall_counter[i] % static_cast <int>(1.0/dt) == 0 ) { // once per 1.0[s]
                          std::cerr << "[" << m_profile.instance_name << "] [" << m_qRef.tm
                                    << "] " << stikp[i].ee_name << " cannot support total weight, "
                                    << "swgsuptime : " << m_controlSwingSupportTime.data[i] << ", state : " << ref_contact_states[i]
                                    << ", otherwise robot will fall down toward " << "(" << projected_normal.at(i)(0) << "," << projected_normal.at(i)(1) << ") direction" << std::endl;
                      }
                      m_will_fall_counter[i]++;
                  } else {
                      m_will_fall_counter[i] = 0;
                  }
              }
              fall_direction += projected_normal.at(i) * act_force.at(i).norm();
              total_force += act_force.at(i).norm();
          }
      }
      if (on_ground && transition_count == 0 && control_mode == MODE_ST) {
          fall_direction = fall_direction / total_force;
      } else {
          fall_direction = hrp::Vector3::Zero();
      }
      if (fall_direction.norm() > sin(tilt_margin[1])) {
          is_falling = true;
          if (m_is_falling_counter % static_cast <int>(0.2/dt) == 0) { // once per 0.2[s]
              std::cerr << "[" << m_profile.instance_name << "] [" << m_qRef.tm
                        << "] robot is falling down toward " << "(" << fall_direction(0) << "," << fall_direction(1) << ") direction" << std::endl;
          }
          m_is_falling_counter++;
      } else {
          m_is_falling_counter = 0;
      }
  }
  // Total check for emergency signal
  switch (emergency_check_mode) {
  case OpenHRP::StabilizerService::NO_CHECK:
      is_emergency = false;
      break;
  case OpenHRP::StabilizerService::COP:
      is_emergency = is_cop_outside && is_seq_interpolating;
      break;
  case OpenHRP::StabilizerService::CP:
      is_emergency = is_cp_outside;
      break;
  case OpenHRP::StabilizerService::TILT:
      is_emergency = will_fall || is_falling;
      break;
  default:
      break;
  }
  if (DEBUGP) {
      std::cerr << "[" << m_profile.instance_name << "] EmergencyCheck ("
                << (emergency_check_mode == OpenHRP::StabilizerService::NO_CHECK?"NO_CHECK": (emergency_check_mode == OpenHRP::StabilizerService::COP?"COP":"CP") )
                << ") " << (is_emergency?"emergency":"non-emergency") << std::endl;
  }
  rel_ee_pos.clear();
  rel_ee_rot.clear();
  rel_ee_name.clear();
};

void Stabilizer::moveBasePosRotForBodyRPYControl ()
{
    // Body rpy control
    //   Basically Equation (1) and (2) in the paper [1]
    hrp::Vector3 ref_root_rpy = hrp::rpyFromRot(target_root_R);
    bool is_root_rot_limit = false;
    for (size_t i = 0; i < 2; i++) {
        d_rpy[i] = transition_smooth_gain * (eefm_body_attitude_control_gain[i] * (ref_root_rpy(i) - act_base_rpy(i)) - 1/eefm_body_attitude_control_time_const[i] * d_rpy[i]) * dt + d_rpy[i];
        d_rpy[i] = vlimit(d_rpy[i], -1 * root_rot_compensation_limit[i], root_rot_compensation_limit[i]);
        is_root_rot_limit = is_root_rot_limit || (std::fabs(std::fabs(d_rpy[i]) - root_rot_compensation_limit[i] ) < 1e-5); // near the limit
    }
    rats::rotm3times(current_root_R, target_root_R, hrp::rotFromRpy(d_rpy[0], d_rpy[1], 0));
    m_robot->rootLink()->R = current_root_R;
    m_robot->rootLink()->p = target_root_p + target_root_R * rel_cog - current_root_R * rel_cog;
    m_robot->calcForwardKinematics();
    current_base_rpy = hrp::rpyFromRot(m_robot->rootLink()->R);
    current_base_pos = m_robot->rootLink()->p;
    if ( DEBUGP || (is_root_rot_limit && loop%200==0) ) {
        std::cerr << "[" << m_profile.instance_name << "] Root rot control" << std::endl;
        if (is_root_rot_limit) std::cerr << "[" << m_profile.instance_name << "]   Root rot limit reached!!" << std::endl;
        std::cerr << "[" << m_profile.instance_name << "]   ref = [" << rad2deg(ref_root_rpy(0)) << " " << rad2deg(ref_root_rpy(1)) << "], "
                  << "act = [" << rad2deg(act_base_rpy(0)) << " " << rad2deg(act_base_rpy(1)) << "], "
                  << "cur = [" << rad2deg(current_base_rpy(0)) << " " << rad2deg(current_base_rpy(1)) << "], "
                  << "limit = [" << rad2deg(root_rot_compensation_limit[0]) << " " << rad2deg(root_rot_compensation_limit[1]) << "][deg]" << std::endl;
    }
};

void Stabilizer::calcSwingSupportLimbGain ()
{
    for (size_t i = 0; i < stikp.size(); i++) {
        STIKParam& ikp = stikp[i];
        if (ref_contact_states[i]) { // Support
            // Limit too large support time increment. Max time is 3600.0[s] = 1[h], this assumes that robot's one step time is smaller than 1[h].
            ikp.support_time = std::min(3600.0, ikp.support_time+dt);
            // In some PC, does not work because the first line is optimized out.
            // ikp.support_time += dt;
            // ikp.support_time = std::min(3600.0, ikp.support_time);
            if (ikp.support_time > eefm_pos_transition_time) {
                ikp.swing_support_gain = (m_controlSwingSupportTime.data[i] / eefm_pos_transition_time);
            } else {
                ikp.swing_support_gain = (ikp.support_time / eefm_pos_transition_time);
            }
            ikp.swing_support_gain = std::max(0.0, std::min(1.0, ikp.swing_support_gain));
        } else { // Swing
            ikp.swing_support_gain = 0.0;
            ikp.support_time = 0.0;
        }
    }
    if (DEBUGP) {
        std::cerr << "[" << m_profile.instance_name << "] SwingSupportLimbGain = [";
        for (size_t i = 0; i < stikp.size(); i++) std::cerr << stikp[i].swing_support_gain << " ";
        std::cerr << "], ref_contact_states = [";
        for (size_t i = 0; i < stikp.size(); i++) std::cerr << ref_contact_states[i] << " ";
        std::cerr << "], sstime = [";
        for (size_t i = 0; i < stikp.size(); i++) std::cerr << m_controlSwingSupportTime.data[i] << " ";
        std::cerr << "], toeheel_ratio = [";
        for (size_t i = 0; i < stikp.size(); i++) std::cerr << toeheel_ratio[i] << " ";
        std::cerr << "], support_time = [";
        for (size_t i = 0; i < stikp.size(); i++) std::cerr << stikp[i].support_time << " ";
        std::cerr << "]" << std::endl;
    }
}

void Stabilizer::calcTPCC() {
    // stabilizer loop
      // Choi's feedback law
      hrp::Vector3 cog = m_robot->calcCM();
      hrp::Vector3 newcog = hrp::Vector3::Zero();
      hrp::Vector3 dcog(ref_cog - act_cog);
      hrp::Vector3 dzmp(ref_zmp - act_zmp);
      for (size_t i = 0; i < 2; i++) {
        double uu = ref_cogvel(i) - k_tpcc_p[i] * transition_smooth_gain * dzmp(i)
                                  + k_tpcc_x[i] * transition_smooth_gain * dcog(i);
        newcog(i) = uu * dt + cog(i);
      }

      moveBasePosRotForBodyRPYControl ();

      // target at ee => target at link-origin
      hrp::Vector3 target_link_p[stikp.size()];
      hrp::Matrix33 target_link_R[stikp.size()];
      for (size_t i = 0; i < stikp.size(); i++) {
        rats::rotm3times(target_link_R[i], target_ee_R[i], stikp[i].localR.transpose());
        target_link_p[i] = target_ee_p[i] - target_ee_R[i] * stikp[i].localCOPPos;
      }
      // solveIK
      //   IK target is link origin pos and rot, not ee pos and rot.
      //for (size_t jj = 0; jj < 5; jj++) {
      size_t max_ik_loop_count = 0;
      for (size_t i = 0; i < stikp.size(); i++) {
          if (max_ik_loop_count < stikp[i].ik_loop_count) max_ik_loop_count = stikp[i].ik_loop_count;
      }
      for (size_t jj = 0; jj < max_ik_loop_count; jj++) {
        hrp::Vector3 tmpcm = m_robot->calcCM();
        for (size_t i = 0; i < 2; i++) {
          m_robot->rootLink()->p(i) = m_robot->rootLink()->p(i) + 0.9 * (newcog(i) - tmpcm(i));
        }
        m_robot->calcForwardKinematics();
        for (size_t i = 0; i < stikp.size(); i++) {
          if (is_ik_enable[i]) {
              jpe_v[i]->calcInverseKinematics2Loop(target_link_p[i], target_link_R[i], 1.0, stikp[i].avoid_gain, stikp[i].reference_gain, &qrefv, transition_smooth_gain);
          }
        }
      }
}


void Stabilizer::calcEEForceMomentControl() {

    // stabilizer loop
      // return to referencea
      m_robot->rootLink()->R = target_root_R;
      m_robot->rootLink()->p = target_root_p;
      for ( int i = 0; i < m_robot->numJoints(); i++ ) {
        m_robot->joint(i)->q = qrefv[i];
      }
      for (size_t i = 0; i < jpe_v.size(); i++) {
        if (is_ik_enable[i]) {
          for ( int j = 0; j < jpe_v[i]->numJoints(); j++ ){
            int idx = jpe_v[i]->joint(j)->jointId;
            m_robot->joint(idx)->q = qorg[idx];
          }
        }
      }
      // Fix for toe joint
      for (size_t i = 0; i < jpe_v.size(); i++) {
          if (is_ik_enable[i]) {
              if (jpe_v[i]->numJoints() == 7) {
                  int idx = jpe_v[i]->joint(jpe_v[i]->numJoints() -1)->jointId;
                  m_robot->joint(idx)->q = qrefv[idx];
              }
          }
      }

      // State calculation for swing ee compensation
      //   joint angle : current control output
      //   root pos : target root p
      //   root rot : actual root rot
      {
          // Calc status
          m_robot->rootLink()->R = target_root_R;
          m_robot->rootLink()->p = target_root_p;
          m_robot->calcForwardKinematics();
          hrp::Sensor* sen = m_robot->sensor<hrp::RateGyroSensor>("gyrometer");
          hrp::Matrix33 senR = sen->link->R * sen->localR;
          hrp::Matrix33 act_Rs(hrp::rotFromRpy(m_rpy.data.r, m_rpy.data.p, m_rpy.data.y));
          m_robot->rootLink()->R = act_Rs * (senR.transpose() * m_robot->rootLink()->R);
          m_robot->calcForwardKinematics();
          hrp::Vector3 foot_origin_pos;
          hrp::Matrix33 foot_origin_rot;
          calcFootOriginCoords (foot_origin_pos, foot_origin_rot);
          // Calculate foot_origin_coords-relative ee pos and rot
          // Subtract them from target_ee_diff_xx
          for (size_t i = 0; i < stikp.size(); i++) {
              hrp::Link* target = m_robot->link(stikp[i].target_name);
              stikp[i].target_ee_diff_p -= foot_origin_rot.transpose() * (target->p + target->R * stikp[i].localp - foot_origin_pos);
              stikp[i].target_ee_diff_r = (foot_origin_rot.transpose() * target->R * stikp[i].localR).transpose() * stikp[i].target_ee_diff_r;
          }
      }

      // State calculation for control : calculate "current" state
      //   joint angle : current control output
      //   root pos : target + keep COG against rpy control
      //   root rot : target + rpy control
      moveBasePosRotForBodyRPYControl ();

      // Convert d_foot_pos in foot origin frame => "current" world frame
      hrp::Vector3 foot_origin_pos;
      hrp::Matrix33 foot_origin_rot;
      calcFootOriginCoords (foot_origin_pos, foot_origin_rot);
      std::vector<hrp::Vector3> current_d_foot_pos;
      for (size_t i = 0; i < stikp.size(); i++)
          current_d_foot_pos.push_back(foot_origin_rot * stikp[i].d_foot_pos);

      // Swing ee compensation.
      calcSwingEEModification();

      // solveIK
      //   IK target is link origin pos and rot, not ee pos and rot.
      std::vector<hrp::Vector3> tmpp(stikp.size());
      std::vector<hrp::Matrix33> tmpR(stikp.size());
      double tmp_d_pos_z_root = 0.0;
      for (size_t i = 0; i < stikp.size(); i++) {
        if (is_ik_enable[i]) {
          // Add damping_control compensation to target value
          if (is_feedback_control_enable[i]) {
            rats::rotm3times(tmpR[i], target_ee_R[i], hrp::rotFromRpy(-1*stikp[i].ee_d_foot_rpy));
            // foot force difference control version
            // total_target_foot_p[i](2) = target_foot_p[i](2) + (i==0?0.5:-0.5)*zctrl;
            // foot force independent damping control
            tmpp[i] = target_ee_p[i] - current_d_foot_pos[i];
          } else {
            tmpp[i] = target_ee_p[i];
            tmpR[i] = target_ee_R[i];
          }
          // Add swing ee compensation
          rats::rotm3times(tmpR[i], tmpR[i], hrp::rotFromRpy(stikp[i].d_rpy_swing));
          tmpp[i] = tmpp[i] + foot_origin_rot * stikp[i].d_pos_swing;
        }
      }

      limbStretchAvoidanceControl(tmpp ,tmpR);

      // IK
      for (size_t i = 0; i < stikp.size(); i++) {
        if (is_ik_enable[i]) {
          for (size_t jj = 0; jj < stikp[i].ik_loop_count; jj++) {
            jpe_v[i]->calcInverseKinematics2Loop(tmpp[i], tmpR[i], 1.0, 0.001, 0.01, &qrefv, transition_smooth_gain,
                                                 //stikp[i].localCOPPos;
                                                 stikp[i].localp,
                                                 stikp[i].localR);
          }
        }
      }
}

// Swing ee compensation.
//   Calculate compensation values to minimize the difference between "current" foot-origin-coords-relative pos and rot and "target" foot-origin-coords-relative pos and rot for swing ee.
//   Input  : target_ee_diff_p, target_ee_diff_r
//   Output : d_pos_swing, d_rpy_swing
void Stabilizer::calcSwingEEModification ()
{
    for (size_t i = 0; i < stikp.size(); i++) {
        // Calc compensation values
        double limit_pos = 30 * 1e-3; // 30[mm] limit
        double limit_rot = deg2rad(10); // 10[deg] limit
        if (ref_contact_states[contact_states_index_map[stikp[i].ee_name]] || act_contact_states[contact_states_index_map[stikp[i].ee_name]]) {
            // If actual contact or target contact is ON, do not use swing ee compensation. Exponential zero retrieving.
            stikp[i].d_rpy_swing = calcDampingControl(stikp[i].d_rpy_swing, stikp[i].eefm_swing_rot_time_const);
            stikp[i].d_pos_swing = calcDampingControl(stikp[i].d_pos_swing, stikp[i].eefm_swing_pos_time_const);
            stikp[i].target_ee_diff_p_filter->reset(stikp[i].d_pos_swing);
            stikp[i].target_ee_diff_r_filter->reset(stikp[i].d_rpy_swing);
        } else {
            /* position */
            {
                hrp::Vector3 tmpdiffp = stikp[i].eefm_swing_pos_spring_gain.cwiseProduct(stikp[i].target_ee_diff_p_filter->passFilter(stikp[i].target_ee_diff_p));
                double lvlimit = -50 * 1e-3 * dt, uvlimit = 50 * 1e-3 * dt; // 50 [mm/s]
                hrp::Vector3 limit_by_lvlimit = stikp[i].prev_d_pos_swing + lvlimit * hrp::Vector3::Ones();
                hrp::Vector3 limit_by_uvlimit = stikp[i].prev_d_pos_swing + uvlimit * hrp::Vector3::Ones();
                stikp[i].d_pos_swing = vlimit(vlimit(tmpdiffp, -1 * limit_pos, limit_pos), limit_by_lvlimit, limit_by_uvlimit);
            }
            /* rotation */
            {
                hrp::Vector3 tmpdiffr = stikp[i].eefm_swing_rot_spring_gain.cwiseProduct(stikp[i].target_ee_diff_r_filter->passFilter(hrp::rpyFromRot(stikp[i].target_ee_diff_r)));
                double lvlimit = deg2rad(-20.0*dt), uvlimit = deg2rad(20.0*dt); // 20 [deg/s]
                hrp::Vector3 limit_by_lvlimit = stikp[i].prev_d_rpy_swing + lvlimit * hrp::Vector3::Ones();
                hrp::Vector3 limit_by_uvlimit = stikp[i].prev_d_rpy_swing + uvlimit * hrp::Vector3::Ones();
                stikp[i].d_rpy_swing = vlimit(vlimit(tmpdiffr, -1 * limit_rot, limit_rot), limit_by_lvlimit, limit_by_uvlimit);
            }
        }
        stikp[i].prev_d_pos_swing = stikp[i].d_pos_swing;
        stikp[i].prev_d_rpy_swing = stikp[i].d_rpy_swing;
    }
    if (DEBUGP) {
        std::cerr << "[" << m_profile.instance_name << "] Swing foot control" << std::endl;
        for (size_t i = 0; i < stikp.size(); i++) {
            std::cerr << "[" << m_profile.instance_name << "]   "
                      << "d_rpy_swing (" << stikp[i].ee_name << ")  = " << (stikp[i].d_rpy_swing / M_PI * 180.0).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "    [", "]")) << "[deg], "
                      << "d_pos_swing (" << stikp[i].ee_name << ")  = " << (stikp[i].d_pos_swing * 1e3).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "    [", "]")) << "[mm]" << std::endl;
        }
    }
};

void Stabilizer::limbStretchAvoidanceControl (const std::vector<hrp::Vector3>& ee_p, const std::vector<hrp::Matrix33>& ee_R)
{
  double tmp_d_pos_z_root = 0.0, prev_d_pos_z_root = d_pos_z_root;
  if (use_limb_stretch_avoidance) {
    for (size_t i = 0; i < stikp.size(); i++) {
      if (is_ik_enable[i]) {
        // Check whether inside limb length limitation
        hrp::Link* parent_link = m_robot->link(stikp[i].parent_name);
        hrp::Vector3 targetp = (ee_p[i] - ee_R[i] * stikp[i].localR.transpose() * stikp[i].localp) - parent_link->p; // position from parent to target link (world frame)
        double limb_length_limitation = stikp[i].max_limb_length - stikp[i].limb_length_margin;
        double tmp = limb_length_limitation * limb_length_limitation - targetp(0) * targetp(0) - targetp(1) * targetp(1);
        if (targetp.norm() > limb_length_limitation && tmp >= 0) {
          tmp_d_pos_z_root = std::min(tmp_d_pos_z_root, targetp(2) + std::sqrt(tmp));
        }
      }
    }
    // Change root link height depending on limb length
    d_pos_z_root = tmp_d_pos_z_root == 0.0 ? calcDampingControl(d_pos_z_root, limb_stretch_avoidance_time_const) : tmp_d_pos_z_root;
  } else {
    d_pos_z_root = calcDampingControl(d_pos_z_root, limb_stretch_avoidance_time_const);
  }
  d_pos_z_root = vlimit(d_pos_z_root, prev_d_pos_z_root + limb_stretch_avoidance_vlimit[0], prev_d_pos_z_root + limb_stretch_avoidance_vlimit[1]);
  m_robot->rootLink()->p(2) += d_pos_z_root;
}

// Damping control functions
//   Basically Equation (14) in the paper [1]
double Stabilizer::calcDampingControl (const double tau_d, const double tau, const double prev_d,
                                       const double DD, const double TT)
{
  return (1/DD * (tau_d - tau) - 1/TT * prev_d) * dt + prev_d;
};

// Retrieving only
hrp::Vector3 Stabilizer::calcDampingControl (const hrp::Vector3& prev_d, const hrp::Vector3& TT)
{
  return (- prev_d.cwiseQuotient(TT)) * dt + prev_d;
};

// Retrieving only
double Stabilizer::calcDampingControl (const double prev_d, const double TT)
{
  return - 1/TT * prev_d * dt + prev_d;
};

hrp::Vector3 Stabilizer::calcDampingControl (const hrp::Vector3& tau_d, const hrp::Vector3& tau, const hrp::Vector3& prev_d,
                                             const hrp::Vector3& DD, const hrp::Vector3& TT)
{
  return ((tau_d - tau).cwiseQuotient(DD) - prev_d.cwiseQuotient(TT)) * dt + prev_d;
};

void Stabilizer::calcDiffFootOriginExtMoment ()
{
    // calc reference ext moment around foot origin pos
    // static const double grav = 9.80665; /* [m/s^2] */
    double mg = total_mass * eefm_gravitational_acceleration;
    hrp::Vector3 ref_ext_moment = hrp::Vector3(mg * ref_cog(1) - ref_total_foot_origin_moment(0),
                                               -mg * ref_cog(0) - ref_total_foot_origin_moment(1),
                                               0);
    // calc act ext moment around foot origin pos
    hrp::Vector3 act_ext_moment = hrp::Vector3(mg * act_cog(1) - act_total_foot_origin_moment(0),
                                               -mg * act_cog(0) - act_total_foot_origin_moment(1),
                                               0);
    // Do not calculate actual value if in the air, because of invalid act_zmp.
    if ( !on_ground ) act_ext_moment = ref_ext_moment;
    // Calc diff
    diff_foot_origin_ext_moment = ref_ext_moment - act_ext_moment;
    if (DEBUGP) {
        std::cerr << "[" << m_profile.instance_name << "] DiffStaticBalancePointOffset" << std::endl;
        std::cerr << "[" << m_profile.instance_name << "]   "
                  << "ref_ext_moment = " << ref_ext_moment.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "    [", "]")) << "[mm], "
                  << "act_ext_moment = " << act_ext_moment.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "    [", "]")) << "[mm], "
                  << "diff ext_moment = " << diff_foot_origin_ext_moment.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "    [", "]")) << "[mm]" << std::endl;
    }
};

/*
RTC::ReturnCode_t Stabilizer::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t Stabilizer::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t Stabilizer::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t Stabilizer::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t Stabilizer::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

void Stabilizer::sync_2_st ()
{
  std::cerr << "[" << m_profile.instance_name << "] [" << m_qRef.tm
            << "] Sync IDLE => ST"  << std::endl;
  pangx_ref = pangy_ref = pangx = pangy = 0;
  rdx = rdy = rx = ry = 0;
  d_rpy[0] = d_rpy[1] = 0;
  pdr = hrp::Vector3::Zero();
  pos_ctrl = hrp::Vector3::Zero();
  for (size_t i = 0; i < stikp.size(); i++) {
    STIKParam& ikp = stikp[i];
    ikp.target_ee_diff_p = hrp::Vector3::Zero();
    ikp.target_ee_diff_r = hrp::Matrix33::Identity();
    ikp.d_pos_swing = ikp.prev_d_pos_swing = hrp::Vector3::Zero();
    ikp.d_rpy_swing = ikp.prev_d_rpy_swing = hrp::Vector3::Zero();
    ikp.target_ee_diff_p_filter->reset(hrp::Vector3::Zero());
    ikp.target_ee_diff_r_filter->reset(hrp::Vector3::Zero());
    ikp.d_foot_pos = ikp.ee_d_foot_pos = ikp.d_foot_rpy = ikp.ee_d_foot_rpy = hrp::Vector3::Zero();
  }
  if (on_ground) {
    transition_count = -1 * calcMaxTransitionCount();
    control_mode = MODE_ST;
  } else {
    transition_count = 0;
    control_mode = MODE_AIR;
  }
}

void Stabilizer::sync_2_idle ()
{
  std::cerr << "[" << m_profile.instance_name << "] [" << m_qRef.tm
            << "] Sync ST => IDLE"  << std::endl;
  transition_count = calcMaxTransitionCount();
  for (int i = 0; i < m_robot->numJoints(); i++ ) {
    transition_joint_q[i] = m_robot->joint(i)->q;
  }
}

void Stabilizer::startStabilizer(void)
{
    waitSTTransition(); // Wait until all transition has finished
    {
        Guard guard(m_mutex);
        if ( control_mode == MODE_IDLE ) {
            std::cerr << "[" << m_profile.instance_name << "] " << "Start ST"  << std::endl;
            sync_2_st();
        }
    }
    waitSTTransition();
    std::cerr << "[" << m_profile.instance_name << "] " << "Start ST DONE"  << std::endl;
}

void Stabilizer::stopStabilizer(void)
{
    waitSTTransition(); // Wait until all transition has finished
    {
        Guard guard(m_mutex);
        if ( (control_mode == MODE_ST || control_mode == MODE_AIR) ) {
            std::cerr << "[" << m_profile.instance_name << "] " << "Stop ST"  << std::endl;
            control_mode = (control_mode == MODE_ST) ? MODE_SYNC_TO_IDLE : MODE_IDLE;
        }
    }
    waitSTTransition();
    std::cerr << "[" << m_profile.instance_name << "] " << "Stop ST DONE"  << std::endl;
}

void Stabilizer::getParameter(OpenHRP::StabilizerService::stParam& i_stp)
{
  std::cerr << "[" << m_profile.instance_name << "] getParameter" << std::endl;
  for (size_t i = 0; i < 2; i++) {
    // i_stp.k_run_b[i] = k_run_b[i];
    // i_stp.d_run_b[i] = d_run_b[i];
    //m_tau_x[i].setup(i_stp.tdfke[0], i_stp.tdftc[0], dt);
    //m_tau_y[i].setup(i_stp.tdfke[0], i_stp.tdftc[0], dt);
    //m_f_z.setup(i_stp.tdfke[1], i_stp.tdftc[1], dt);
    i_stp.k_tpcc_p[i] = k_tpcc_p[i];
    i_stp.k_tpcc_x[i] = k_tpcc_x[i];
    i_stp.k_brot_p[i] = k_brot_p[i];
    i_stp.k_brot_tc[i] = k_brot_tc[i];
  }
  // i_stp.k_run_x = m_torque_k[0];
  // i_stp.k_run_y = m_torque_k[1];
  // i_stp.d_run_x = m_torque_d[0];
  // i_stp.d_run_y = m_torque_d[1];
  for (size_t i = 0; i < 2; i++) {
    i_stp.eefm_k1[i] = eefm_k1[i];
    i_stp.eefm_k2[i] = eefm_k2[i];
    i_stp.eefm_k3[i] = eefm_k3[i];
    i_stp.eefm_zmp_delay_time_const[i] = eefm_zmp_delay_time_const[i];
    i_stp.eefm_ref_zmp_aux[i] = ref_zmp_aux(i);
    i_stp.eefm_body_attitude_control_time_const[i] = eefm_body_attitude_control_time_const[i];
    i_stp.eefm_body_attitude_control_gain[i] = eefm_body_attitude_control_gain[i];
    i_stp.ref_capture_point[i] = ref_cp(i);
    i_stp.act_capture_point[i] = act_cp(i);
    i_stp.cp_offset[i] = cp_offset(i);
  }
  i_stp.eefm_pos_time_const_support.length(stikp.size());
  i_stp.eefm_pos_damping_gain.length(stikp.size());
  i_stp.eefm_pos_compensation_limit.length(stikp.size());
  i_stp.eefm_swing_pos_spring_gain.length(stikp.size());
  i_stp.eefm_swing_pos_time_const.length(stikp.size());
  i_stp.eefm_rot_time_const.length(stikp.size());
  i_stp.eefm_rot_damping_gain.length(stikp.size());
  i_stp.eefm_rot_compensation_limit.length(stikp.size());
  i_stp.eefm_swing_rot_spring_gain.length(stikp.size());
  i_stp.eefm_swing_rot_time_const.length(stikp.size());
  i_stp.eefm_ee_moment_limit.length(stikp.size());
  i_stp.eefm_ee_forcemoment_distribution_weight.length(stikp.size());
  for (size_t j = 0; j < stikp.size(); j++) {
      i_stp.eefm_pos_damping_gain[j].length(3);
      i_stp.eefm_pos_time_const_support[j].length(3);
      i_stp.eefm_swing_pos_spring_gain[j].length(3);
      i_stp.eefm_swing_pos_time_const[j].length(3);
      i_stp.eefm_rot_damping_gain[j].length(3);
      i_stp.eefm_rot_time_const[j].length(3);
      i_stp.eefm_swing_rot_spring_gain[j].length(3);
      i_stp.eefm_swing_rot_time_const[j].length(3);
      i_stp.eefm_ee_moment_limit[j].length(3);
      i_stp.eefm_ee_forcemoment_distribution_weight[j].length(6);
      for (size_t i = 0; i < 3; i++) {
          i_stp.eefm_pos_damping_gain[j][i] = stikp[j].eefm_pos_damping_gain(i);
          i_stp.eefm_pos_time_const_support[j][i] = stikp[j].eefm_pos_time_const_support(i);
          i_stp.eefm_swing_pos_spring_gain[j][i] = stikp[j].eefm_swing_pos_spring_gain(i);
          i_stp.eefm_swing_pos_time_const[j][i] = stikp[j].eefm_swing_pos_time_const(i);
          i_stp.eefm_rot_damping_gain[j][i] = stikp[j].eefm_rot_damping_gain(i);
          i_stp.eefm_rot_time_const[j][i] = stikp[j].eefm_rot_time_const(i);
          i_stp.eefm_swing_rot_spring_gain[j][i] = stikp[j].eefm_swing_rot_spring_gain(i);
          i_stp.eefm_swing_rot_time_const[j][i] = stikp[j].eefm_swing_rot_time_const(i);
          i_stp.eefm_ee_moment_limit[j][i] = stikp[j].eefm_ee_moment_limit(i);
          i_stp.eefm_ee_forcemoment_distribution_weight[j][i] = stikp[j].eefm_ee_forcemoment_distribution_weight(i);
          i_stp.eefm_ee_forcemoment_distribution_weight[j][i+3] = stikp[j].eefm_ee_forcemoment_distribution_weight(i+3);
      }
      i_stp.eefm_pos_compensation_limit[j] = stikp[j].eefm_pos_compensation_limit;
      i_stp.eefm_rot_compensation_limit[j] = stikp[j].eefm_rot_compensation_limit;
  }
  for (size_t i = 0; i < 3; i++) {
    i_stp.eefm_swing_pos_damping_gain[i] = eefm_swing_pos_damping_gain(i);
    i_stp.eefm_swing_rot_damping_gain[i] = eefm_swing_rot_damping_gain(i);
  }
  i_stp.eefm_pos_time_const_swing = eefm_pos_time_const_swing;
  i_stp.eefm_pos_transition_time = eefm_pos_transition_time;
  i_stp.eefm_pos_margin_time = eefm_pos_margin_time;
  i_stp.eefm_leg_inside_margin = szd->get_leg_inside_margin();
  i_stp.eefm_leg_outside_margin = szd->get_leg_outside_margin();
  i_stp.eefm_leg_front_margin = szd->get_leg_front_margin();
  i_stp.eefm_leg_rear_margin = szd->get_leg_rear_margin();

  std::vector<std::vector<Eigen::Vector2d> > support_polygon_vec;
  szd->get_vertices(support_polygon_vec);
  i_stp.eefm_support_polygon_vertices_sequence.length(support_polygon_vec.size());
  for (size_t ee_idx = 0; ee_idx < support_polygon_vec.size(); ee_idx++) {
      i_stp.eefm_support_polygon_vertices_sequence[ee_idx].vertices.length(support_polygon_vec[ee_idx].size());
      for (size_t v_idx = 0; v_idx < support_polygon_vec[ee_idx].size(); v_idx++) {
          i_stp.eefm_support_polygon_vertices_sequence[ee_idx].vertices[v_idx].pos[0] = support_polygon_vec[ee_idx][v_idx](0);
          i_stp.eefm_support_polygon_vertices_sequence[ee_idx].vertices[v_idx].pos[1] = support_polygon_vec[ee_idx][v_idx](1);
      }
  }

  i_stp.eefm_cogvel_cutoff_freq = act_cogvel_filter->getCutOffFreq();
  i_stp.eefm_wrench_alpha_blending = szd->get_wrench_alpha_blending();
  i_stp.eefm_alpha_cutoff_freq = szd->get_alpha_cutoff_freq();
  i_stp.eefm_gravitational_acceleration = eefm_gravitational_acceleration;
  i_stp.eefm_ee_error_cutoff_freq = stikp[0].target_ee_diff_p_filter->getCutOffFreq();
  i_stp.eefm_use_force_difference_control = eefm_use_force_difference_control;
  i_stp.eefm_use_swing_damping = eefm_use_swing_damping;
  for (size_t i = 0; i < 3; ++i) {
      i_stp.eefm_swing_damping_force_thre[i] = eefm_swing_damping_force_thre[i];
      i_stp.eefm_swing_damping_moment_thre[i] = eefm_swing_damping_moment_thre[i];
  }
  i_stp.is_ik_enable.length(is_ik_enable.size());
  for (size_t i = 0; i < is_ik_enable.size(); i++) {
      i_stp.is_ik_enable[i] = is_ik_enable[i];
  }
  i_stp.is_feedback_control_enable.length(is_feedback_control_enable.size());
  for (size_t i = 0; i < is_feedback_control_enable.size(); i++) {
      i_stp.is_feedback_control_enable[i] = is_feedback_control_enable[i];
  }
  i_stp.is_zmp_calc_enable.length(is_zmp_calc_enable.size());
  for (size_t i = 0; i < is_zmp_calc_enable.size(); i++) {
      i_stp.is_zmp_calc_enable[i] = is_zmp_calc_enable[i];
  }

  i_stp.foot_origin_offset.length(2);
  for (size_t i = 0; i < i_stp.foot_origin_offset.length(); i++) {
      i_stp.foot_origin_offset[i].length(3);
      i_stp.foot_origin_offset[i][0] = foot_origin_offset[i](0);
      i_stp.foot_origin_offset[i][1] = foot_origin_offset[i](1);
      i_stp.foot_origin_offset[i][2] = foot_origin_offset[i](2);
  }
  i_stp.st_algorithm = st_algorithm;
  i_stp.transition_time = transition_time;
  i_stp.cop_check_margin = cop_check_margin;
  for (size_t i = 0; i < cp_check_margin.size(); i++) {
    i_stp.cp_check_margin[i] = cp_check_margin[i];
  }
  for (size_t i = 0; i < tilt_margin.size(); i++) {
    i_stp.tilt_margin[i] = tilt_margin[i];
  }
  i_stp.contact_decision_threshold = contact_decision_threshold;
  i_stp.is_estop_while_walking = is_estop_while_walking;
  switch(control_mode) {
  case MODE_IDLE: i_stp.controller_mode = OpenHRP::StabilizerService::MODE_IDLE; break;
  case MODE_AIR: i_stp.controller_mode = OpenHRP::StabilizerService::MODE_AIR; break;
  case MODE_ST: i_stp.controller_mode = OpenHRP::StabilizerService::MODE_ST; break;
  case MODE_SYNC_TO_IDLE: i_stp.controller_mode = OpenHRP::StabilizerService::MODE_SYNC_TO_IDLE; break;
  case MODE_SYNC_TO_AIR: i_stp.controller_mode = OpenHRP::StabilizerService::MODE_SYNC_TO_AIR; break;
  default: break;
  }
  i_stp.emergency_check_mode = emergency_check_mode;
  i_stp.end_effector_list.length(stikp.size());
  i_stp.use_limb_stretch_avoidance = use_limb_stretch_avoidance;
  i_stp.use_zmp_truncation = use_zmp_truncation;
  i_stp.limb_stretch_avoidance_time_const = limb_stretch_avoidance_time_const;
  i_stp.limb_length_margin.length(stikp.size());
  i_stp.detection_time_to_air = detection_count_to_air * dt;
  for (size_t i = 0; i < 2; i++) {
    i_stp.limb_stretch_avoidance_vlimit[i] = limb_stretch_avoidance_vlimit[i];
    i_stp.root_rot_compensation_limit[i] = root_rot_compensation_limit[i];
  }
  for (size_t i = 0; i < stikp.size(); i++) {
      const rats::coordinates cur_ee = rats::coordinates(stikp.at(i).localp, stikp.at(i).localR);
      OpenHRP::AutoBalancerService::Footstep ret_ee;
      // position
      memcpy(ret_ee.pos, cur_ee.pos.data(), sizeof(double)*3);
      // rotation
      Eigen::Quaternion<double> qt(cur_ee.rot);
      ret_ee.rot[0] = qt.w();
      ret_ee.rot[1] = qt.x();
      ret_ee.rot[2] = qt.y();
      ret_ee.rot[3] = qt.z();
      // name
      ret_ee.leg = stikp.at(i).ee_name.c_str();
      // set
      i_stp.end_effector_list[i] = ret_ee;
      i_stp.limb_length_margin[i] = stikp[i].limb_length_margin;
  }
  i_stp.ik_limb_parameters.length(jpe_v.size());
  for (size_t i = 0; i < jpe_v.size(); i++) {
      OpenHRP::StabilizerService::IKLimbParameters& ilp = i_stp.ik_limb_parameters[i];
      ilp.ik_optional_weight_vector.length(jpe_v[i]->numJoints());
      std::vector<double> ov;
      ov.resize(jpe_v[i]->numJoints());
      jpe_v[i]->getOptionalWeightVector(ov);
      for (size_t j = 0; j < jpe_v[i]->numJoints(); j++) {
          ilp.ik_optional_weight_vector[j] = ov[j];
      }
      ilp.sr_gain = jpe_v[i]->getSRGain();
      ilp.avoid_gain = stikp[i].avoid_gain;
      ilp.reference_gain = stikp[i].reference_gain;
      ilp.manipulability_limit = jpe_v[i]->getManipulabilityLimit();
      ilp.ik_loop_count = stikp[i].ik_loop_count; // size_t -> unsigned short, value may change, but ik_loop_count is small value and value not change
  }
};

void Stabilizer::setParameter(const OpenHRP::StabilizerService::stParam& i_stp)
{
  Guard guard(m_mutex);
  std::cerr << "[" << m_profile.instance_name << "] setParameter" << std::endl;
  for (size_t i = 0; i < 2; i++) {
    k_tpcc_p[i] = i_stp.k_tpcc_p[i];
    k_tpcc_x[i] = i_stp.k_tpcc_x[i];
    k_brot_p[i] = i_stp.k_brot_p[i];
    k_brot_tc[i] = i_stp.k_brot_tc[i];
  }
  std::cerr << "[" << m_profile.instance_name << "]  TPCC" << std::endl;
  std::cerr << "[" << m_profile.instance_name << "]   k_tpcc_p  = [" << k_tpcc_p[0] << ", " <<  k_tpcc_p[1] << "], k_tpcc_x  = [" << k_tpcc_x[0] << ", " << k_tpcc_x[1] << "], k_brot_p  = [" << k_brot_p[0] << ", " << k_brot_p[1] << "], k_brot_tc = [" << k_brot_tc[0] << ", " << k_brot_tc[1] << "]" << std::endl;
  // for (size_t i = 0; i < 2; i++) {
  //   k_run_b[i] = i_stp.k_run_b[i];
  //   d_run_b[i] = i_stp.d_run_b[i];
  //   m_tau_x[i].setup(i_stp.tdfke[0], i_stp.tdftc[0], dt);
  //   m_tau_y[i].setup(i_stp.tdfke[0], i_stp.tdftc[0], dt);
  //   m_f_z.setup(i_stp.tdfke[1], i_stp.tdftc[1], dt);
  // }
  // m_torque_k[0] = i_stp.k_run_x;
  // m_torque_k[1] = i_stp.k_run_y;
  // m_torque_d[0] = i_stp.d_run_x;
  // m_torque_d[1] = i_stp.d_run_y;
  // std::cerr << "[" << m_profile.instance_name << "]  RUNST" << std::endl;
  // std::cerr << "[" << m_profile.instance_name << "]   m_torque_k  = [" << m_torque_k[0] << ", " <<  m_torque_k[1] << "]" << std::endl;
  // std::cerr << "[" << m_profile.instance_name << "]   m_torque_d  = [" << m_torque_d[0] << ", " <<  m_torque_d[1] << "]" << std::endl;
  // std::cerr << "[" << m_profile.instance_name << "]   k_run_b  = [" << k_run_b[0] << ", " <<  k_run_b[1] << "]" << std::endl;
  // std::cerr << "[" << m_profile.instance_name << "]   d_run_b  = [" << d_run_b[0] << ", " <<  d_run_b[1] << "]" << std::endl;
  std::cerr << "[" << m_profile.instance_name << "]  EEFM" << std::endl;
  for (size_t i = 0; i < 2; i++) {
    eefm_k1[i] = i_stp.eefm_k1[i];
    eefm_k2[i] = i_stp.eefm_k2[i];
    eefm_k3[i] = i_stp.eefm_k3[i];
    eefm_zmp_delay_time_const[i] = i_stp.eefm_zmp_delay_time_const[i];
    ref_zmp_aux(i) = i_stp.eefm_ref_zmp_aux[i];
    eefm_body_attitude_control_gain[i] = i_stp.eefm_body_attitude_control_gain[i];
    eefm_body_attitude_control_time_const[i] = i_stp.eefm_body_attitude_control_time_const[i];
    ref_cp(i) = i_stp.ref_capture_point[i];
    act_cp(i) = i_stp.act_capture_point[i];
    cp_offset(i) = i_stp.cp_offset[i];
  }
  bool is_damping_parameter_ok = true;
  if ( i_stp.eefm_pos_damping_gain.length () == stikp.size() &&
       i_stp.eefm_pos_time_const_support.length () == stikp.size() &&
       i_stp.eefm_pos_compensation_limit.length () == stikp.size() &&
       i_stp.eefm_swing_pos_spring_gain.length () == stikp.size() &&
       i_stp.eefm_swing_pos_time_const.length () == stikp.size() &&
       i_stp.eefm_rot_damping_gain.length () == stikp.size() &&
       i_stp.eefm_rot_time_const.length () == stikp.size() &&
       i_stp.eefm_rot_compensation_limit.length () == stikp.size() &&
       i_stp.eefm_swing_rot_spring_gain.length () == stikp.size() &&
       i_stp.eefm_swing_rot_time_const.length () == stikp.size() &&
       i_stp.eefm_ee_moment_limit.length () == stikp.size() &&
       i_stp.eefm_ee_forcemoment_distribution_weight.length () == stikp.size()) {
      is_damping_parameter_ok = true;
      for (size_t j = 0; j < stikp.size(); j++) {
          for (size_t i = 0; i < 3; i++) {
              stikp[j].eefm_pos_damping_gain(i) = i_stp.eefm_pos_damping_gain[j][i];
              stikp[j].eefm_pos_time_const_support(i) = i_stp.eefm_pos_time_const_support[j][i];
              stikp[j].eefm_swing_pos_spring_gain(i) = i_stp.eefm_swing_pos_spring_gain[j][i];
              stikp[j].eefm_swing_pos_time_const(i) = i_stp.eefm_swing_pos_time_const[j][i];
              stikp[j].eefm_rot_damping_gain(i) = i_stp.eefm_rot_damping_gain[j][i];
              stikp[j].eefm_rot_time_const(i) = i_stp.eefm_rot_time_const[j][i];
              stikp[j].eefm_swing_rot_spring_gain(i) = i_stp.eefm_swing_rot_spring_gain[j][i];
              stikp[j].eefm_swing_rot_time_const(i) = i_stp.eefm_swing_rot_time_const[j][i];
              stikp[j].eefm_ee_moment_limit(i) = i_stp.eefm_ee_moment_limit[j][i];
              stikp[j].eefm_ee_forcemoment_distribution_weight(i) = i_stp.eefm_ee_forcemoment_distribution_weight[j][i];
              stikp[j].eefm_ee_forcemoment_distribution_weight(i+3) = i_stp.eefm_ee_forcemoment_distribution_weight[j][i+3];
          }
          stikp[j].eefm_pos_compensation_limit = i_stp.eefm_pos_compensation_limit[j];
          stikp[j].eefm_rot_compensation_limit = i_stp.eefm_rot_compensation_limit[j];
      }
  } else {
      is_damping_parameter_ok = false;
  }
  for (size_t i = 0; i < 3; i++) {
    eefm_swing_pos_damping_gain(i) = i_stp.eefm_swing_pos_damping_gain[i];
    eefm_swing_rot_damping_gain(i) = i_stp.eefm_swing_rot_damping_gain[i];
  }
  eefm_pos_time_const_swing = i_stp.eefm_pos_time_const_swing;
  eefm_pos_transition_time = i_stp.eefm_pos_transition_time;
  eefm_pos_margin_time = i_stp.eefm_pos_margin_time;
  szd->set_leg_inside_margin(i_stp.eefm_leg_inside_margin);
  szd->set_leg_outside_margin(i_stp.eefm_leg_outside_margin);
  szd->set_leg_front_margin(i_stp.eefm_leg_front_margin);
  szd->set_leg_rear_margin(i_stp.eefm_leg_rear_margin);
  szd->set_vertices_from_margin_params();

  if (i_stp.eefm_support_polygon_vertices_sequence.length() != stikp.size()) {
      std::cerr << "[" << m_profile.instance_name << "]   eefm_support_polygon_vertices_sequence cannot be set. Length " << i_stp.eefm_support_polygon_vertices_sequence.length() << " != " << stikp.size() << std::endl;
  } else {
      std::cerr << "[" << m_profile.instance_name << "]   eefm_support_polygon_vertices_sequence set" << std::endl;
      std::vector<std::vector<Eigen::Vector2d> > support_polygon_vec;
      for (size_t ee_idx = 0; ee_idx < i_stp.eefm_support_polygon_vertices_sequence.length(); ee_idx++) {
          std::vector<Eigen::Vector2d> tvec;
          for (size_t v_idx = 0; v_idx < i_stp.eefm_support_polygon_vertices_sequence[ee_idx].vertices.length(); v_idx++) {
              tvec.push_back(Eigen::Vector2d(i_stp.eefm_support_polygon_vertices_sequence[ee_idx].vertices[v_idx].pos[0],
                                             i_stp.eefm_support_polygon_vertices_sequence[ee_idx].vertices[v_idx].pos[1]));
          }
          support_polygon_vec.push_back(tvec);
      }
      szd->set_vertices(support_polygon_vec);
      szd->print_vertices(std::string(m_profile.instance_name));
  }
  eefm_use_force_difference_control = i_stp.eefm_use_force_difference_control;
  eefm_use_swing_damping = i_stp.eefm_use_swing_damping;
  for (size_t i = 0; i < 3; ++i) {
      eefm_swing_damping_force_thre[i] = i_stp.eefm_swing_damping_force_thre[i];
      eefm_swing_damping_moment_thre[i] = i_stp.eefm_swing_damping_moment_thre[i];
  }
  act_cogvel_filter->setCutOffFreq(i_stp.eefm_cogvel_cutoff_freq);
  szd->set_wrench_alpha_blending(i_stp.eefm_wrench_alpha_blending);
  szd->set_alpha_cutoff_freq(i_stp.eefm_alpha_cutoff_freq);
  eefm_gravitational_acceleration = i_stp.eefm_gravitational_acceleration;
  for (size_t i = 0; i < stikp.size(); i++) {
      stikp[i].target_ee_diff_p_filter->setCutOffFreq(i_stp.eefm_ee_error_cutoff_freq);
      stikp[i].target_ee_diff_r_filter->setCutOffFreq(i_stp.eefm_ee_error_cutoff_freq);
      stikp[i].limb_length_margin = i_stp.limb_length_margin[i];
  }
  setBoolSequenceParam(is_ik_enable, i_stp.is_ik_enable, std::string("is_ik_enable"));
  setBoolSequenceParamWithCheckContact(is_feedback_control_enable, i_stp.is_feedback_control_enable, std::string("is_feedback_control_enable"));
  setBoolSequenceParam(is_zmp_calc_enable, i_stp.is_zmp_calc_enable, std::string("is_zmp_calc_enable"));
  emergency_check_mode = i_stp.emergency_check_mode;

  transition_time = i_stp.transition_time;
  cop_check_margin = i_stp.cop_check_margin;
  for (size_t i = 0; i < cp_check_margin.size(); i++) {
    cp_check_margin[i] = i_stp.cp_check_margin[i];
  }
  szd->set_vertices_from_margin_params(cp_check_margin);
  for (size_t i = 0; i < tilt_margin.size(); i++) {
    tilt_margin[i] = i_stp.tilt_margin[i];
  }
  contact_decision_threshold = i_stp.contact_decision_threshold;
  is_estop_while_walking = i_stp.is_estop_while_walking;
  use_limb_stretch_avoidance = i_stp.use_limb_stretch_avoidance;
  use_zmp_truncation = i_stp.use_zmp_truncation;
  limb_stretch_avoidance_time_const = i_stp.limb_stretch_avoidance_time_const;
  for (size_t i = 0; i < 2; i++) {
    limb_stretch_avoidance_vlimit[i] = i_stp.limb_stretch_avoidance_vlimit[i];
    root_rot_compensation_limit[i] = i_stp.root_rot_compensation_limit[i];
  }
  detection_count_to_air = static_cast<int>(i_stp.detection_time_to_air / dt);
  if (control_mode == MODE_IDLE) {
      for (size_t i = 0; i < i_stp.end_effector_list.length(); i++) {
          std::vector<STIKParam>::iterator it = std::find_if(stikp.begin(), stikp.end(), (&boost::lambda::_1->* &std::vector<STIKParam>::value_type::ee_name == std::string(i_stp.end_effector_list[i].leg)));
          memcpy(it->localp.data(), i_stp.end_effector_list[i].pos, sizeof(double)*3);
          it->localR = (Eigen::Quaternion<double>(i_stp.end_effector_list[i].rot[0], i_stp.end_effector_list[i].rot[1], i_stp.end_effector_list[i].rot[2], i_stp.end_effector_list[i].rot[3])).normalized().toRotationMatrix();
      }
  } else {
      std::cerr << "[" << m_profile.instance_name << "] cannot change end-effectors except during MODE_IDLE" << std::endl;
  }
  for (std::vector<STIKParam>::const_iterator it = stikp.begin(); it != stikp.end(); it++) {
      std::cerr << "[" << m_profile.instance_name << "]  End Effector [" << it->ee_name << "]" << std::endl;
      std::cerr << "[" << m_profile.instance_name << "]   localpos = " << it->localp.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "    [", "]")) << "[m]" << std::endl;
      std::cerr << "[" << m_profile.instance_name << "]   localR = " << it->localR.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", "", "    [", "]")) << std::endl;
  }
  if (i_stp.foot_origin_offset.length () != 2) {
      std::cerr << "[" << m_profile.instance_name << "]   foot_origin_offset cannot be set. Length " << i_stp.foot_origin_offset.length() << " != " << 2 << std::endl;
  } else if (control_mode != MODE_IDLE) {
      std::cerr << "[" << m_profile.instance_name << "]   foot_origin_offset cannot be set. Current control_mode is " << control_mode << std::endl;
  } else {
      for (size_t i = 0; i < i_stp.foot_origin_offset.length(); i++) {
          foot_origin_offset[i](0) = i_stp.foot_origin_offset[i][0];
          foot_origin_offset[i](1) = i_stp.foot_origin_offset[i][1];
          foot_origin_offset[i](2) = i_stp.foot_origin_offset[i][2];
      }
  }
  std::cerr << "[" << m_profile.instance_name << "]   foot_origin_offset is ";
  for (size_t i = 0; i < 2; i++) {
      std::cerr << foot_origin_offset[i].format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "    [", "]"));
  }
  std::cerr << "[m]" << std::endl;
  std::cerr << "[" << m_profile.instance_name << "]   eefm_k1  = [" << eefm_k1[0] << ", " << eefm_k1[1] << "], eefm_k2  = [" << eefm_k2[0] << ", " << eefm_k2[1] << "], eefm_k3  = [" << eefm_k3[0] << ", " << eefm_k3[1] << "]" << std::endl;
  std::cerr << "[" << m_profile.instance_name << "]   eefm_zmp_delay_time_const  = [" << eefm_zmp_delay_time_const[0] << ", " << eefm_zmp_delay_time_const[1] << "][s], eefm_ref_zmp_aux  = [" << ref_zmp_aux(0) << ", " << ref_zmp_aux(1) << "][m]" << std::endl;
  std::cerr << "[" << m_profile.instance_name << "]   eefm_body_attitude_control_gain  = [" << eefm_body_attitude_control_gain[0] << ", " << eefm_body_attitude_control_gain[1] << "], eefm_body_attitude_control_time_const  = [" << eefm_body_attitude_control_time_const[0] << ", " << eefm_body_attitude_control_time_const[1] << "][s]" << std::endl;
  if (is_damping_parameter_ok) {
      for (size_t j = 0; j < stikp.size(); j++) {
          std::cerr << "[" << m_profile.instance_name << "]   [" << stikp[j].ee_name << "] eefm_rot_damping_gain = "
                    << stikp[j].eefm_rot_damping_gain.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "    [", "]"))
                    << ", eefm_rot_time_const = "
                    << stikp[j].eefm_rot_time_const.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "    [", "]"))
                    << "[s]" << std::endl;
          std::cerr << "[" << m_profile.instance_name << "]   [" << stikp[j].ee_name << "] eefm_pos_damping_gain = "
                    << stikp[j].eefm_pos_damping_gain.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "    [", "]"))
                    << ", eefm_pos_time_const_support = "
                    << stikp[j].eefm_pos_time_const_support.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "    [", "]"))
                    << "[s]" << std::endl;
          std::cerr << "[" << m_profile.instance_name << "]   [" << stikp[j].ee_name << "] "
                    << "eefm_pos_compensation_limit = " << stikp[j].eefm_pos_compensation_limit << "[m], "
                    << "eefm_rot_compensation_limit = " << stikp[j].eefm_rot_compensation_limit << "[rad], "
                    << "eefm_ee_moment_limit = " << stikp[j].eefm_ee_moment_limit.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "    [", "]")) << "[Nm]" << std::endl;
          std::cerr << "[" << m_profile.instance_name << "]   [" << stikp[j].ee_name << "] "
                    << "eefm_swing_pos_spring_gain = " << stikp[j].eefm_swing_pos_spring_gain.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "    [", "]")) << ", "
                    << "eefm_swing_pos_time_const = " << stikp[j].eefm_swing_pos_time_const.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "    [", "]")) << ", "
                    << "eefm_swing_rot_spring_gain = " << stikp[j].eefm_swing_rot_spring_gain.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "    [", "]")) << ", "
                    << "eefm_swing_pos_time_const = " << stikp[j].eefm_swing_pos_time_const.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "    [", "]")) << ", "
                    << std::endl;
          std::cerr << "[" << m_profile.instance_name << "]   [" << stikp[j].ee_name << "] "
                    << "eefm_ee_forcemoment_distribution_weight = " << stikp[j].eefm_ee_forcemoment_distribution_weight.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "    [", "]")) << "" << std::endl;
      }
  } else {
      std::cerr << "[" << m_profile.instance_name << "]   eefm damping parameters cannot be set because of invalid param." << std::endl;
  }
  std::cerr << "[" << m_profile.instance_name << "]   eefm_pos_transition_time = " << eefm_pos_transition_time << "[s], eefm_pos_margin_time = " << eefm_pos_margin_time << "[s] eefm_pos_time_const_swing = " << eefm_pos_time_const_swing << "[s]" << std::endl;
  std::cerr << "[" << m_profile.instance_name << "]   cogvel_cutoff_freq = " << act_cogvel_filter->getCutOffFreq() << "[Hz]" << std::endl;
  szd->print_params(std::string(m_profile.instance_name));
  std::cerr << "[" << m_profile.instance_name << "]   eefm_gravitational_acceleration = " << eefm_gravitational_acceleration << "[m/s^2], eefm_use_force_difference_control = " << (eefm_use_force_difference_control? "true":"false") << ", eefm_use_swing_damping = " << (eefm_use_swing_damping? "true":"false") << std::endl;
  std::cerr << "[" << m_profile.instance_name << "]   eefm_ee_error_cutoff_freq = " << stikp[0].target_ee_diff_p_filter->getCutOffFreq() << "[Hz]" << std::endl;
  std::cerr << "[" << m_profile.instance_name << "]  COMMON" << std::endl;
  if (control_mode == MODE_IDLE) {
    st_algorithm = i_stp.st_algorithm;
    std::cerr << "[" << m_profile.instance_name << "]   st_algorithm changed to [" << getStabilizerAlgorithmString(st_algorithm) << "]" << std::endl;
  } else {
    std::cerr << "[" << m_profile.instance_name << "]   st_algorithm cannot be changed to [" << getStabilizerAlgorithmString(st_algorithm) << "] during MODE_AIR or MODE_ST." << std::endl;
  }
  std::cerr << "[" << m_profile.instance_name << "]   emergency_check_mode changed to [" << (emergency_check_mode == OpenHRP::StabilizerService::NO_CHECK?"NO_CHECK": (emergency_check_mode == OpenHRP::StabilizerService::COP?"COP":"CP") ) << "]" << std::endl;
  std::cerr << "[" << m_profile.instance_name << "]   transition_time = " << transition_time << "[s]" << std::endl;
  std::cerr << "[" << m_profile.instance_name << "]   cop_check_margin = " << cop_check_margin << "[m], "
            << "cp_check_margin = [" << cp_check_margin[0] << ", " << cp_check_margin[1] << ", " << cp_check_margin[2] << ", " << cp_check_margin[3] << "] [m], "
            << "tilt_margin = [" << tilt_margin[0] << ", " << tilt_margin[1] << "] [rad]" << std::endl;
  std::cerr << "[" << m_profile.instance_name << "]   contact_decision_threshold = " << contact_decision_threshold << "[N], detection_time_to_air = " << detection_count_to_air * dt << "[s]" << std::endl;
  std::cerr << "[" << m_profile.instance_name << "]   root_rot_compensation_limit = [" << root_rot_compensation_limit[0] << " " << root_rot_compensation_limit[1] << "][rad]" << std::endl;
  // IK limb parameters
  std::cerr << "[" << m_profile.instance_name << "]  IK limb parameters" << std::endl;
  bool is_ik_limb_parameter_valid_length = true;
  if (i_stp.ik_limb_parameters.length() != jpe_v.size()) {
      is_ik_limb_parameter_valid_length = false;
      std::cerr << "[" << m_profile.instance_name << "]   ik_limb_parameters invalid length! Cannot be set. (input = " << i_stp.ik_limb_parameters.length() << ", desired = " << jpe_v.size() << ")" << std::endl;
  } else {
      for (size_t i = 0; i < jpe_v.size(); i++) {
          if (jpe_v[i]->numJoints() != i_stp.ik_limb_parameters[i].ik_optional_weight_vector.length())
              is_ik_limb_parameter_valid_length = false;
      }
      if (is_ik_limb_parameter_valid_length) {
          for (size_t i = 0; i < jpe_v.size(); i++) {
              const OpenHRP::StabilizerService::IKLimbParameters& ilp = i_stp.ik_limb_parameters[i];
              std::vector<double> ov;
              ov.resize(jpe_v[i]->numJoints());
              for (size_t j = 0; j < jpe_v[i]->numJoints(); j++) {
                  ov[j] = ilp.ik_optional_weight_vector[j];
              }
              jpe_v[i]->setOptionalWeightVector(ov);
              jpe_v[i]->setSRGain(ilp.sr_gain);
              stikp[i].avoid_gain = ilp.avoid_gain;
              stikp[i].reference_gain = ilp.reference_gain;
              jpe_v[i]->setManipulabilityLimit(ilp.manipulability_limit);
              stikp[i].ik_loop_count = ilp.ik_loop_count; // unsigned short -> size_t, value not change
          }
      } else {
          std::cerr << "[" << m_profile.instance_name << "]   ik_optional_weight_vector invalid length! Cannot be set. (input = [";
          for (size_t i = 0; i < jpe_v.size(); i++) {
              std::cerr << i_stp.ik_limb_parameters[i].ik_optional_weight_vector.length() << ", ";
          }
          std::cerr << "], desired = [";
          for (size_t i = 0; i < jpe_v.size(); i++) {
              std::cerr << jpe_v[i]->numJoints() << ", ";
          }
          std::cerr << "])" << std::endl;
      }
  }
  if (is_ik_limb_parameter_valid_length) {
      std::cerr << "[" << m_profile.instance_name << "]   ik_optional_weight_vectors = ";
      for (size_t i = 0; i < jpe_v.size(); i++) {
          std::vector<double> ov;
          ov.resize(jpe_v[i]->numJoints());
          jpe_v[i]->getOptionalWeightVector(ov);
          std::cerr << "[";
          for (size_t j = 0; j < jpe_v[i]->numJoints(); j++) {
              std::cerr << ov[j] << " ";
          }
          std::cerr << "]";
      }
      std::cerr << std::endl;
      std::cerr << "[" << m_profile.instance_name << "]   sr_gains = [";
      for (size_t i = 0; i < jpe_v.size(); i++) {
          std::cerr << jpe_v[i]->getSRGain() << ", ";
      }
      std::cerr << "]" << std::endl;
      std::cerr << "[" << m_profile.instance_name << "]   avoid_gains = [";
      for (size_t i = 0; i < stikp.size(); i++) {
          std::cerr << stikp[i].avoid_gain << ", ";
      }
      std::cerr << "]" << std::endl;
      std::cerr << "[" << m_profile.instance_name << "]   reference_gains = [";
      for (size_t i = 0; i < stikp.size(); i++) {
          std::cerr << stikp[i].reference_gain << ", ";
      }
      std::cerr << "]" << std::endl;
      std::cerr << "[" << m_profile.instance_name << "]   manipulability_limits = [";
      for (size_t i = 0; i < jpe_v.size(); i++) {
          std::cerr << jpe_v[i]->getManipulabilityLimit() << ", ";
      }
      std::cerr << "]" << std::endl;
      std::cerr << "[" << m_profile.instance_name << "]   ik_loop_count = [";
      for (size_t i = 0; i < stikp.size(); i++) {
          std::cerr << stikp[i].ik_loop_count << ", ";
      }
      std::cerr << "]" << std::endl;
  }
}

std::string Stabilizer::getStabilizerAlgorithmString (OpenHRP::StabilizerService::STAlgorithm _st_algorithm)
{
    switch (_st_algorithm) {
    case OpenHRP::StabilizerService::TPCC:
        return "TPCC";
    case OpenHRP::StabilizerService::EEFM:
        return "EEFM";
    case OpenHRP::StabilizerService::EEFMQP:
        return "EEFMQP";
    case OpenHRP::StabilizerService::EEFMQPCOP:
        return "EEFMQPCOP";
    case OpenHRP::StabilizerService::EEFMQPCOP2:
        return "EEFMQPCOP2";
    default:
        return "";
    }
};

void Stabilizer::setBoolSequenceParam (std::vector<bool>& st_bool_values, const OpenHRP::StabilizerService::BoolSequence& output_bool_values, const std::string& prop_name)
{
  std::vector<bool> prev_values;
  prev_values.resize(st_bool_values.size());
  copy (st_bool_values.begin(), st_bool_values.end(), prev_values.begin());
  if (st_bool_values.size() != output_bool_values.length()) {
      std::cerr << "[" << m_profile.instance_name << "]   " << prop_name << " cannot be set. Length " << st_bool_values.size() << " != " << output_bool_values.length() << std::endl;
  } else if ( (control_mode != MODE_IDLE) ) {
      std::cerr << "[" << m_profile.instance_name << "]   " << prop_name << " cannot be set. Current control_mode is " << control_mode << std::endl;
  } else {
      for (size_t i = 0; i < st_bool_values.size(); i++) {
          st_bool_values[i] = output_bool_values[i];
      }
  }
  std::cerr << "[" << m_profile.instance_name << "]   " << prop_name << " is ";
  for (size_t i = 0; i < st_bool_values.size(); i++) {
      std::cerr <<"[" << st_bool_values[i] << "]";
  }
  std::cerr << "(set = ";
  for (size_t i = 0; i < output_bool_values.length(); i++) {
      std::cerr <<"[" << output_bool_values[i] << "]";
  }
  std::cerr << ", prev = ";
  for (size_t i = 0; i < prev_values.size(); i++) {
      std::cerr <<"[" << prev_values[i] << "]";
  }
  std::cerr << ")" << std::endl;
};

void Stabilizer::setBoolSequenceParamWithCheckContact (std::vector<bool>& st_bool_values, const OpenHRP::StabilizerService::BoolSequence& output_bool_values, const std::string& prop_name)
{
  std::vector<bool> prev_values;
  prev_values.resize(st_bool_values.size());
  copy (st_bool_values.begin(), st_bool_values.end(), prev_values.begin());
  if (st_bool_values.size() != output_bool_values.length()) {
      std::cerr << "[" << m_profile.instance_name << "]   " << prop_name << " cannot be set. Length " << st_bool_values.size() << " != " << output_bool_values.length() << std::endl;
  } else if ( control_mode == MODE_IDLE ) {
    for (size_t i = 0; i < st_bool_values.size(); i++) {
      st_bool_values[i] = output_bool_values[i];
    }
  } else {
    std::vector<size_t> failed_indices;
    for (size_t i = 0; i < st_bool_values.size(); i++) {
      if ( (st_bool_values[i] != output_bool_values[i]) ) { // If mode change
        if (!ref_contact_states[i] ) { // reference contact_states should be OFF
          st_bool_values[i] = output_bool_values[i];
        } else {
          failed_indices.push_back(i);
        }
      }
    }
    if (failed_indices.size() > 0) {
      std::cerr << "[" << m_profile.instance_name << "]   " << prop_name << " cannot be set partially. failed_indices is [";
      for (size_t i = 0; i < failed_indices.size(); i++) {
        std::cerr << failed_indices[i] << " ";
      }
      std::cerr << "]" << std::endl;
    }
  }
  std::cerr << "[" << m_profile.instance_name << "]   " << prop_name << " is ";
  for (size_t i = 0; i < st_bool_values.size(); i++) {
      std::cerr <<"[" << st_bool_values[i] << "]";
  }
  std::cerr << "(set = ";
  for (size_t i = 0; i < output_bool_values.length(); i++) {
      std::cerr <<"[" << output_bool_values[i] << "]";
  }
  std::cerr << ", prev = ";
  for (size_t i = 0; i < prev_values.size(); i++) {
      std::cerr <<"[" << prev_values[i] << "]";
  }
  std::cerr << ")" << std::endl;
};

void Stabilizer::waitSTTransition()
{
  // Wait condition
  //   1. Check transition_count : Wait until transition is finished
  //   2. Check control_mode : Once control_mode is SYNC mode, wait until control_mode moves to the next mode (MODE_AIR or MODE_IDLE)
  bool flag = (control_mode == MODE_SYNC_TO_AIR || control_mode == MODE_SYNC_TO_IDLE);
  while (transition_count != 0 ||
         (flag ? !(control_mode == MODE_IDLE || control_mode == MODE_AIR) : false) ) {
      usleep(10);
      flag = (control_mode == MODE_SYNC_TO_AIR || control_mode == MODE_SYNC_TO_IDLE);
  }
  usleep(10);
}

double Stabilizer::vlimit(double value, double llimit_value, double ulimit_value)
{
  if (value > ulimit_value) {
    return ulimit_value;
  } else if (value < llimit_value) {
    return llimit_value;
  }
  return value;
}

hrp::Vector3 Stabilizer::vlimit(const hrp::Vector3& value, double llimit_value, double ulimit_value)
{
  hrp::Vector3 ret;
  for (size_t i = 0; i < 3; i++) {
      if (value(i) > ulimit_value) {
          ret(i) = ulimit_value;
      } else if (value(i) < llimit_value) {
          ret(i) = llimit_value;
      } else {
          ret(i) = value(i);
      }
  }
  return ret;
}

hrp::Vector3 Stabilizer::vlimit(const hrp::Vector3& value, const hrp::Vector3& limit_value)
{
  hrp::Vector3 ret;
  for (size_t i = 0; i < 3; i++) {
      if (value(i) > limit_value(i)) {
          ret(i) = limit_value(i);
      } else if (value(i) < -1 * limit_value(i)) {
          ret(i) = -1 * limit_value(i);
      } else {
          ret(i) = value(i);
      }
  }
  return ret;
}

hrp::Vector3 Stabilizer::vlimit(const hrp::Vector3& value, const hrp::Vector3& llimit_value, const hrp::Vector3& ulimit_value)
{
  hrp::Vector3 ret;
  for (size_t i = 0; i < 3; i++) {
      if (value(i) > ulimit_value(i)) {
          ret(i) = ulimit_value(i);
      } else if (value(i) < llimit_value(i)) {
          ret(i) = llimit_value(i);
      } else {
          ret(i) = value(i);
      }
  }
  return ret;
}

static double switching_inpact_absorber(double force, double lower_th, double upper_th)
{
  double gradient, intercept;
  if (force < lower_th) {
    return 0;
  } else if (force > upper_th) {
    return 1;
  } else {
    gradient = 1.0 / (upper_th - lower_th);
    intercept = -lower_th * gradient;
    return gradient * force + intercept;
  }
}

void Stabilizer::calcRUNST() {
  if ( m_robot->numJoints() == m_qRef.data.length() ) {
    std::vector<std::string> target_name;
    target_name.push_back("L_ANKLE_R");
    target_name.push_back("R_ANKLE_R");

    double angvelx_ref;// = (m_rpyRef.data.r - pangx_ref)/dt;
    double angvely_ref;// = (m_rpyRef.data.p - pangy_ref)/dt;
    //pangx_ref = m_rpyRef.data.r;
    //pangy_ref = m_rpyRef.data.p;
    double angvelx = (m_rpy.data.r - pangx)/dt;
    double angvely = (m_rpy.data.r - pangy)/dt;
    pangx = m_rpy.data.r;
    pangy = m_rpy.data.p;

    // update internal robot model
    for ( int i = 0; i < m_robot->numJoints(); i++ ){
      qorg[i] = m_robot->joint(i)->q;
      m_robot->joint(i)->q = m_qRef.data[i];
      qrefv[i] = m_qRef.data[i];
    }
    //double orgjq = m_robot->link("L_FOOT")->joint->q;
    double orgjq = m_robot->joint(m_robot->link("L_ANKLE_P")->jointId)->q;
    //set root
    m_robot->rootLink()->p = hrp::Vector3(0,0,0);
    //m_robot->rootLink()->R = hrp::rotFromRpy(m_rpyRef.data.r,m_rpyRef.data.p,m_rpyRef.data.y);
    m_robot->calcForwardKinematics();
    hrp::Vector3 target_root_p = m_robot->rootLink()->p;
    hrp::Matrix33 target_root_R = m_robot->rootLink()->R;
    hrp::Vector3 target_foot_p[2];
    hrp::Matrix33 target_foot_R[2];
    for (size_t i = 0; i < 2; i++) {
      target_foot_p[i] = m_robot->link(target_name[i])->p;
      target_foot_R[i] = m_robot->link(target_name[i])->R;
    }
    hrp::Vector3 target_fm = (m_robot->link(target_name[0])->p + m_robot->link(target_name[1])->p)/2;
    //hrp::Vector3 org_cm = m_robot->rootLink()->R.transpose() * (m_robot->calcCM() - m_robot->rootLink()->p);
    hrp::Vector3 org_cm = m_robot->rootLink()->R.transpose() * (target_fm - m_robot->rootLink()->p);

    // stabilizer loop
    if ( ( m_wrenches[1].data.length() > 0 && m_wrenches[0].data.length() > 0 )
         //( m_wrenches[ST_LEFT].data[2] > m_robot->totalMass()/4 || m_wrenches[ST_RIGHT].data[2] > m_robot->totalMass()/4 )
         ) {

      for ( int i = 0; i < m_robot->numJoints(); i++ ) {
        m_robot->joint(i)->q = qorg[i];
      }
      // set root
      double rddx;// = k_run_b[0] * (m_rpyRef.data.r - m_rpy.data.r) + d_run_b[0] * (angvelx_ref - angvelx);
      double rddy;// = k_run_b[1] * (m_rpyRef.data.p - m_rpy.data.p) + d_run_b[1] * (angvely_ref - angvely);
      rdx += rddx * dt;
      rx += rdx * dt;
      rdy += rddy * dt;
      ry += rdy * dt;
      //rx += rddx * dt;
      //ry += rddy * dt;
      // if (DEBUGP2) {
      //   std::cerr << "REFRPY " <<  m_rpyRef.data.r << " " << m_rpyRef.data.p << std::endl;
      // }
      // if (DEBUGP2) {
      //   std::cerr << "RPY " <<  m_rpy.data.r << " " << m_rpy.data.p << std::endl;
      //   std::cerr << " rx " << rx << " " << rdx << " " << rddx << std::endl;
      //   std::cerr << " ry " << ry << " " << rdy << " " << rddy << std::endl;
      // }
      hrp::Vector3 root_p_s;
      hrp::Matrix33 root_R_s;
      rats::rotm3times(root_R_s, hrp::rotFromRpy(rx, ry, 0), target_root_R);
      if (DEBUGP2) {
        hrp::Vector3 tmp = hrp::rpyFromRot(root_R_s);
        std::cerr << "RPY2 " <<  tmp(0) << " " << tmp(1) << std::endl;
      }
      root_p_s = target_root_p + target_root_R * org_cm - root_R_s * org_cm;
      //m_robot->calcForwardKinematics();
      // FK
      m_robot->rootLink()->R = root_R_s;
      m_robot->rootLink()->p = root_p_s;
      if (DEBUGP2) {
        std::cerr << " rp " << root_p_s[0] << " " << root_p_s[1] << " " << root_p_s[2] << std::endl;
      }
      m_robot->calcForwardKinematics();
      //
      hrp::Vector3 current_fm = (m_robot->link(target_name[0])->p + m_robot->link(target_name[1])->p)/2;

      // 3D-LIP model contorller
      hrp::Vector3 dr = target_fm - current_fm;
      //hrp::Vector3 dr = current_fm - target_fm ;
      hrp::Vector3 dr_vel = (dr - pdr)/dt;
      pdr = dr;
      double tau_y = - m_torque_k[0] * dr(0) - m_torque_d[0] * dr_vel(0);
      double tau_x = m_torque_k[1] * dr(1) + m_torque_d[1] * dr_vel(1);
      if (DEBUGP2) {
        dr*=1e3;
        dr_vel*=1e3;
        std::cerr << "dr " << dr(0) << " " << dr(1) << " " << dr_vel(0) << " " << dr_vel(1) << std::endl;
        std::cerr << "tau_x " << tau_x << std::endl;
        std::cerr << "tau_y " << tau_y << std::endl;
      }

      double gamma = 0.5; // temp
      double tau_xl[2];
      double tau_yl[2];
      double xfront = 0.125;
      double xrear = 0.1;
      double yin = 0.02;
      double yout = 0.15;
      double mg = m_robot->totalMass() * 9.8 * 0.9;// margin
      double tq_y_ulimit = mg * xrear;
      double tq_y_llimit = -1 * mg * xfront;
      double tq_x_ulimit = mg * yout;
      double tq_x_llimit = mg * yin;
      // left
      tau_xl[0] = gamma * tau_x;
      tau_yl[0] = gamma * tau_y;
      tau_xl[0] = vlimit(tau_xl[0], tq_x_llimit, tq_x_ulimit);
      tau_yl[0] = vlimit(tau_yl[0], tq_y_llimit, tq_y_ulimit);
      // right
      tau_xl[1]= (1- gamma) * tau_x;
      tau_yl[1]= (1- gamma) * tau_y;
      tau_xl[1] = vlimit(tau_xl[1], -1*tq_x_ulimit, -1*tq_x_llimit);
      tau_yl[1] = vlimit(tau_yl[1], tq_y_llimit, tq_y_ulimit);

      double dleg_x[2];
      double dleg_y[2];
      double tau_y_total = (m_wrenches[1].data[4] + m_wrenches[0].data[4]) / 2;
      double dpz;
      if (DEBUGP2) {
        std::cerr << "tq limit " << tq_x_ulimit << " " << tq_x_llimit << " " << tq_y_ulimit << " " << tq_y_llimit << std::endl;
      }
      for (size_t i = 0; i < 2; i++) {
        // dleg_x[i] = m_tau_x[i].update(m_wrenches[i].data[3], tau_xl[i]);
        // dleg_y[i] = m_tau_y[i].update(m_wrenches[i].data[4], tau_yl[i]);
        //dleg_x[i] = m_tau_x[i].update(m_wrenches[i].data[3], tau_xl[i]);
        dleg_x[i] = m_tau_x[i].update(0,0);
        dleg_y[i] = m_tau_y[i].update(tau_y_total, tau_yl[i]);
        if (DEBUGP2) {
          std::cerr << i << " dleg_x " << dleg_x[i] << std::endl;
          std::cerr << i << " dleg_y " << dleg_y[i] << std::endl;
          std::cerr << i << " t_x " << m_wrenches[i].data[3] << " "<< tau_xl[i] << std::endl;
          std::cerr << i << " t_y " << m_wrenches[i].data[4] << " "<< tau_yl[i] << std::endl;
        }
      }

      // calc leg rot
      hrp::Matrix33 target_R[2];
      hrp::Vector3 target_p[2];
      for (size_t i = 0; i < 2; i++) {
        //rats::rotm3times(target_R[i], hrp::rotFromRpy(dleg_x[i], dleg_y[i], 0), target_foot_R[i]);
        rats::rotm3times(target_R[i], hrp::rotFromRpy(0, dleg_y[i], 0), target_foot_R[i]);
        //target_p[i] = target_foot_p[i] + target_foot_R[i] * org_cm - target_R[i] * org_cm;
        //target_p[i] = target_foot_p[i] + target_foot_R[i] * org_cm - target_R[i] * org_cm;
        target_p[i] = target_foot_p[i];
      }
      // 1=>left, 2=>right
      double refdfz = 0;
      dpz = m_f_z.update((m_wrenches[0].data[2] - m_wrenches[1].data[2]), refdfz);
      //target_p[0](2) = target_foot_p[0](2) + dpz/2;
      //target_p[1](2) = target_foot_p[1](2) - dpz/2;
      target_p[0](2) = target_foot_p[0](2);
      target_p[1](2) = target_foot_p[1](2);

      // IK
      for (size_t i = 0; i < 2; i++) {
        hrp::Link* target = m_robot->link(target_name[i]);
        hrp::Vector3 vel_p, vel_r;
        vel_p = target_p[i] - target->p;
        rats::difference_rotation(vel_r, target->R, target_R[i]);
        //jpe_v[i]->solveLimbIK(vel_p, vel_r, transition_count, 0.001, 0.01, MAX_TRANSITION_COUNT, qrefv, DEBUGP);
        //jpe_v[i]->solveLimbIK(vel_p, vel_r, transition_count, 0.001, 0.01, MAX_TRANSITION_COUNT, qrefv, false);
        //m_robot->joint(m_robot->link(target_name[i])->jointId)->q = dleg_y[i] + orgjq;
      }
      // m_robot->joint(m_robot->link("L_ANKLE_P")->jointId)->q = transition_smooth_gain * dleg_y[0] + orgjq + m_rpy.data.p;
      // m_robot->joint(m_robot->link("R_ANKLE_P")->jointId)->q = transition_smooth_gain * dleg_y[1] + orgjq + m_rpy.data.p;
      m_robot->joint(m_robot->link("L_ANKLE_P")->jointId)->q = transition_smooth_gain * dleg_y[0] + orgjq;
      m_robot->joint(m_robot->link("R_ANKLE_P")->jointId)->q = transition_smooth_gain * dleg_y[1] + orgjq;
    } else {
      // reinitialize
      for (int i = 0; i < 2; i++) {
        m_tau_x[i].reset();
        m_tau_y[i].reset();
        m_f_z.reset();
      }
    }
  }
}

void Stabilizer::calcContactMatrix (hrp::dmatrix& tm, const std::vector<hrp::Vector3>& contact_p)
{
  // tm.resize(6,6*contact_p.size());
  // tm.setZero();
  // for (size_t c = 0; c < contact_p.size(); c++) {
  //   for (size_t i = 0; i < 6; i++) tm(i,(c*6)+i) = 1.0;
  //   hrp::Matrix33 cm;
  //   rats::outer_product_matrix(cm, contact_p[c]);
  //   for (size_t i = 0; i < 3; i++)
  //     for (size_t j = 0; j < 3; j++) tm(i+3,(c*6)+j) = cm(i,j);
  // }
}

void Stabilizer::calcTorque ()
{
  m_robot->calcForwardKinematics();
  // buffers for the unit vector method
  hrp::Vector3 root_w_x_v;
  hrp::Vector3 g(0, 0, 9.80665);
  root_w_x_v = m_robot->rootLink()->w.cross(m_robot->rootLink()->vo + m_robot->rootLink()->w.cross(m_robot->rootLink()->p));
  m_robot->rootLink()->dvo = g - root_w_x_v;   // dv = g, dw = 0
  m_robot->rootLink()->dw.setZero();

  hrp::Vector3 root_f;
  hrp::Vector3 root_t;
  m_robot->calcInverseDynamics(m_robot->rootLink(), root_f, root_t);
  // if (loop % 200 == 0) {
  //   std::cerr << ":mass " << m_robot->totalMass() << std::endl;
  //   std::cerr << ":cog "; rats::print_vector(std::cerr, m_robot->calcCM());
  //   for(int i = 0; i < m_robot->numJoints(); ++i){
  //     std::cerr << "(list :" << m_robot->link(i)->name << " "
  //               << m_robot->joint(i)->jointId << " "
  //               << m_robot->link(i)->m << " ";
  //     hrp::Vector3 tmpc = m_robot->link(i)->p + m_robot->link(i)->R * m_robot->link(i)->c;
  //     rats::print_vector(std::cerr, tmpc, false);
  //     std::cerr << " ";
  //     rats::print_vector(std::cerr, m_robot->link(i)->c, false);
  //     std::cerr << ")" << std::endl;
  //   }
  // }
  // if (loop % 200 == 0) {
  //   std::cerr << ":IV1 (list ";
  //   for(int i = 0; i < m_robot->numJoints(); ++i){
  //     std::cerr << "(list :" << m_robot->joint(i)->name << " " <<  m_robot->joint(i)->u << ")";
  //   }
  //   std::cerr << ")" << std::endl;
  // }
  hrp::dmatrix contact_mat, contact_mat_inv;
  std::vector<hrp::Vector3> contact_p;
  for (size_t j = 0; j < 2; j++) contact_p.push_back(m_robot->sensor<hrp::ForceSensor>(stikp[j].sensor_name)->link->p);
  calcContactMatrix(contact_mat, contact_p);
  hrp::calcSRInverse(contact_mat, contact_mat_inv, 0.0);
  hrp::dvector root_ft(6);
  for (size_t j = 0; j < 3; j++) root_ft(j) = root_f(j);
  for (size_t j = 0; j < 3; j++) root_ft(j+3) = root_t(j);
  hrp::dvector contact_ft(2*6);
  contact_ft = contact_mat_inv * root_ft;
  // if (loop%200==0) {
  //   std::cerr << ":mass " << m_robot->totalMass() << std::endl;
  //   // std::cerr << ":ftv "; rats::print_vector(std::cerr, ftv);
  //   // std::cerr << ":aa "; rats::print_matrix(std::cerr, aa);
  //   // std::cerr << ":dv "; rats::print_vector(std::cerr, dv);
  // }
  for (size_t j = 0; j < 2; j++) {
    hrp::JointPathEx jm = hrp::JointPathEx(m_robot, m_robot->rootLink(), m_robot->sensor<hrp::ForceSensor>(stikp[j].sensor_name)->link, dt);
    hrp::dmatrix JJ;
    jm.calcJacobian(JJ);
    hrp::dvector ft(6);
    for (size_t i = 0; i < 6; i++) ft(i) = contact_ft(i+j*6);
    hrp::dvector tq_from_extft(jm.numJoints());
    tq_from_extft = JJ.transpose() * ft;
    // if (loop%200==0) {
    //   std::cerr << ":ft "; rats::print_vector(std::cerr, ft);
    //   std::cerr << ":JJ "; rats::print_matrix(std::cerr, JJ);
    //   std::cerr << ":tq_from_extft "; rats::print_vector(std::cerr, tq_from_extft);
    // }
    for (size_t i = 0; i < jm.numJoints(); i++) jm.joint(i)->u -= tq_from_extft(i);
  }
  //hrp::dmatrix MM(6,m_robot->numJoints());
  //m_robot->calcMassMatrix(MM);
  // if (loop % 200 == 0) {
  //   std::cerr << ":INVDYN2 (list "; rats::print_vector(std::cerr, root_f, false);
  //   std::cerr << " "; rats::print_vector(std::cerr, root_t, false);
  //   std::cerr << ")" << std::endl;
  //   // hrp::dvector tqv(m_robot->numJoints());
  //   // for(int i = 0; i < m_robot->numJoints(); ++i){p
  //   //   tqv[m_robot->joint(i)->jointId] = m_robot->joint(i)->u;
  //   // }
  //   // std::cerr << ":IV2 "; rats::print_vector(std::cerr, tqv);
  //   std::cerr << ":IV2 (list ";
  //   for(int i = 0; i < m_robot->numJoints(); ++i){
  //     std::cerr << "(list :" << m_robot->joint(i)->name << " " <<  m_robot->joint(i)->u << ")";
  //   }
  //   std::cerr << ")" << std::endl;
  // }
};

extern "C"
{

  void StabilizerInit(RTC::Manager* manager)
  {
    RTC::Properties profile(stabilizer_spec);
    manager->registerFactory(profile,
                             RTC::Create<Stabilizer>,
                             RTC::Delete<Stabilizer>);
  }

};


