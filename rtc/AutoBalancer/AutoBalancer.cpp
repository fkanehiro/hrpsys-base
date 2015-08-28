// -*- C++ -*-
/*!
 * @file  AutoBalancer.cpp
 * @brief autobalancer component
 * $Date$
 *
 * $Id$
 */

#include <rtm/CorbaNaming.h>
#include <hrpModel/Link.h>
#include <hrpModel/Sensor.h>
#include <hrpModel/ModelLoaderUtil.h>
#include "AutoBalancer.h"
#include <hrpModel/JointPath.h>
#include <hrpUtil/MatrixSolvers.h>
#include "util/Hrpsys.h"


typedef coil::Guard<coil::Mutex> Guard;
using namespace rats;

// Module specification
// <rtc-template block="module_spec">
static const char* autobalancer_spec[] =
    {
        "implementation_id", "AutoBalancer",
        "type_name",         "AutoBalancer",
        "description",       "autobalancer component",
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

AutoBalancer::AutoBalancer(RTC::Manager* manager)
    : RTC::DataFlowComponentBase(manager),
      // <rtc-template block="initializer">
      m_qRefIn("qRef", m_qRef),
      m_basePosIn("basePosIn", m_basePos),
      m_baseRpyIn("baseRpyIn", m_baseRpy),
      m_zmpIn("zmpIn", m_zmp),
      m_optionalDataIn("optionalData", m_optionalData),
      m_emergencySignalIn("emergencySignal", m_emergencySignal),
      m_qOut("q", m_qRef),
      m_zmpOut("zmpOut", m_zmp),
      m_basePosOut("basePosOut", m_basePos),
      m_baseRpyOut("baseRpyOut", m_baseRpy),
      m_baseTformOut("baseTformOut", m_baseTform),
      m_basePoseOut("basePoseOut", m_basePose),
      m_accRefOut("accRef", m_accRef),
      m_contactStatesOut("contactStates", m_contactStates),
      m_controlSwingSupportTimeOut("controlSwingSupportTime", m_controlSwingSupportTime),
      m_cogOut("cogOut", m_cog),
      m_AutoBalancerServicePort("AutoBalancerService"),
      // </rtc-template>
      move_base_gain(0.8),
      m_robot(hrp::BodyPtr()),
      m_debugLevel(0)
{
    m_service0.autobalancer(this);
}

AutoBalancer::~AutoBalancer()
{
}


RTC::ReturnCode_t AutoBalancer::onInitialize()
{
    std::cerr << "[" << m_profile.instance_name << "] onInitialize()" << std::endl;
    bindParameter("debugLevel", m_debugLevel, "0");

    // Registration: InPort/OutPort/Service
    // <rtc-template block="registration">
    // Set InPort buffers
    addInPort("qRef", m_qRefIn);
    addInPort("basePosIn", m_basePosIn);
    addInPort("baseRpyIn", m_baseRpyIn);
    addInPort("zmpIn", m_zmpIn);
    addInPort("optionalData", m_optionalDataIn);
    addInPort("emergencySignal", m_emergencySignalIn);

    // Set OutPort buffer
    addOutPort("q", m_qOut);
    addOutPort("zmpOut", m_zmpOut);
    addOutPort("basePosOut", m_basePosOut);
    addOutPort("baseRpyOut", m_baseRpyOut);
    addOutPort("baseTformOut", m_baseTformOut);
    addOutPort("basePoseOut", m_basePoseOut);
    addOutPort("accRef", m_accRefOut);
    addOutPort("contactStates", m_contactStatesOut);
    addOutPort("controlSwingSupportTime", m_controlSwingSupportTimeOut);
    addOutPort("cogOut", m_cogOut);
  
    // Set service provider to Ports
    m_AutoBalancerServicePort.registerProvider("service0", "AutoBalancerService", m_service0);
  
    // Set service consumers to Ports
  
    // Set CORBA Service Ports
    addPort(m_AutoBalancerServicePort);
  
    // </rtc-template>
    // <rtc-template block="bind_config">
    // Bind variables and configuration variable
  
    // </rtc-template>

    RTC::Properties& prop =  getProperties();
    coil::stringTo(m_dt, prop["dt"].c_str());

    m_robot = hrp::BodyPtr(new hrp::Body());
    RTC::Manager& rtcManager = RTC::Manager::instance();
    std::string nameServer = rtcManager.getConfig()["corba.nameservers"];
    int comPos = nameServer.find(",");
    if (comPos < 0){
        comPos = nameServer.length();
    }
    nameServer = nameServer.substr(0, comPos);
    RTC::CorbaNaming naming(rtcManager.getORB(), nameServer.c_str());
    if (!loadBodyFromModelLoader(m_robot, prop["model"].c_str(), 
                                 CosNaming::NamingContext::_duplicate(naming.getRootContext())
                                 )){
      std::cerr << m_profile.instance_name << " failed to load model[" << prop["model"] << "]" << std::endl;
      return RTC::RTC_ERROR;
    }

    // allocate memory for outPorts
    m_qRef.data.length(m_robot->numJoints());
    qorg.resize(m_robot->numJoints());
    qrefv.resize(m_robot->numJoints());
    m_baseTform.data.length(12);

    control_mode = MODE_IDLE;
    loop = 0;

    zmp_interpolator = new interpolator(6, m_dt);
    zmp_transition_time = 1.0;
    transition_interpolator = new interpolator(1, m_dt, interpolator::HOFFARBIB, 1);
    transition_interpolator_ratio = 1.0;
    adjust_footstep_interpolator = new interpolator(1, m_dt, interpolator::HOFFARBIB, 1);
    transition_time = 2.0;
    adjust_footstep_transition_time = 2.0;

    // setting from conf file
    // GaitGenerator requires abc_leg_offset and abc_stride_parameter in robot conf file
    // setting leg_pos from conf file
    coil::vstring leg_offset_str = coil::split(prop["abc_leg_offset"], ",");
    std::vector<hrp::Vector3> leg_pos;
    if (leg_offset_str.size() > 0) {
      hrp::Vector3 leg_offset;
      for (size_t i = 0; i < 3; i++) coil::stringTo(leg_offset(i), leg_offset_str[i].c_str());
      std::cerr << "[" << m_profile.instance_name << "] abc_leg_offset = " << leg_offset.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "    [", "]")) << "[m]" << std::endl;
      leg_pos.push_back(hrp::Vector3(-1*leg_offset));
      leg_pos.push_back(hrp::Vector3(leg_offset));
    }
    leg_names.push_back("rleg");
    leg_names.push_back("lleg");
    // setting stride limitations from conf file
    double stride_fwd_x_limit = 0.15;
    double stride_y_limit = 0.05;
    double stride_th_limit = 10;
    double stride_bwd_x_limit = 0.05;
    std::cerr << "[" << m_profile.instance_name << "] abc_stride_parameter : " << stride_fwd_x_limit << "[m], " << stride_y_limit << "[m], " << stride_th_limit << "[deg], " << stride_bwd_x_limit << "[m]" << std::endl;
    if (default_zmp_offsets.size() == 0) {
      for (size_t i = 0; i < leg_pos.size(); i++) default_zmp_offsets.push_back(hrp::Vector3::Zero());
    }
    if (leg_offset_str.size() > 0) {
      gg = ggPtr(new rats::gait_generator(m_dt, leg_pos, leg_names, stride_fwd_x_limit/*[m]*/, stride_y_limit/*[m]*/, stride_th_limit/*[deg]*/, stride_bwd_x_limit/*[m]*/));
      gg->set_default_zmp_offsets(default_zmp_offsets);
    }
    gg_is_walking = gg_solved = false;
    fix_leg_coords = coordinates();

    // setting from conf file
    // rleg,TARGET_LINK,BASE_LINK
    coil::vstring end_effectors_str = coil::split(prop["end_effectors"], ",");
    size_t prop_num = 10;
    if (end_effectors_str.size() > 0) {
      size_t num = end_effectors_str.size()/prop_num;
      for (size_t i = 0; i < num; i++) {
        std::string ee_name, ee_target, ee_base;
        coil::stringTo(ee_name, end_effectors_str[i*prop_num].c_str());
        coil::stringTo(ee_target, end_effectors_str[i*prop_num+1].c_str());
        coil::stringTo(ee_base, end_effectors_str[i*prop_num+2].c_str());
        ABCIKparam tp;
        for (size_t j = 0; j < 3; j++) {
          coil::stringTo(tp.localPos(j), end_effectors_str[i*prop_num+3+j].c_str());
        }
        double tmpv[4];
        for (int j = 0; j < 4; j++ ) {
          coil::stringTo(tmpv[j], end_effectors_str[i*prop_num+6+j].c_str());
        }
        tp.localR = Eigen::AngleAxis<double>(tmpv[3], hrp::Vector3(tmpv[0], tmpv[1], tmpv[2])).toRotationMatrix(); // rotation in VRML is represented by axis + angle
        tp.manip = hrp::JointPathExPtr(new hrp::JointPathEx(m_robot, m_robot->link(ee_base), m_robot->link(ee_target), m_dt, false));
        // Fix for toe joint
        if (ee_name.find("leg") != std::string::npos && tp.manip->numJoints() == 7) { // leg and 7dof joint (6dof leg +1dof toe)
            std::vector<double> optw;
            for (int j = 0; j < tp.manip->numJoints(); j++ ) {
                if ( j == tp.manip->numJoints()-1 ) optw.push_back(0.0);
                else optw.push_back(1.0);
            }
            tp.manip->setOptionalWeightVector(optw);
        }
        ikp.insert(std::pair<std::string, ABCIKparam>(ee_name , tp));
        ikp[ee_name].target_link = m_robot->link(ee_target);
        std::cerr << "[" << m_profile.instance_name << "] End Effector [" << ee_name << "]" << std::endl;
        std::cerr << "[" << m_profile.instance_name << "]   target = " << ikp[ee_name].target_link->name << ", base = " << ee_base << std::endl;
        std::cerr << "[" << m_profile.instance_name << "]   offset_pos = " << tp.localPos.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "    [", "]")) << "[m]" << std::endl;
        contact_states_index_map.insert(std::pair<std::string, size_t>(ee_name, i));
      }
      m_contactStates.data.length(num);
      if (ikp.find("rleg") != ikp.end() && ikp.find("lleg") != ikp.end()) {
        m_contactStates.data[contact_states_index_map["rleg"]] = true;
        m_contactStates.data[contact_states_index_map["lleg"]] = true;
      }
      if (ikp.find("rarm") != ikp.end() && ikp.find("larm") != ikp.end()) {
        m_contactStates.data[contact_states_index_map["rarm"]] = false;
        m_contactStates.data[contact_states_index_map["larm"]] = false;
      }
      m_controlSwingSupportTime.data.length(num);
      for (size_t i = 0; i < num; i++) m_controlSwingSupportTime.data[i] = 0.0;
    }

    // load virtual force sensors
    readVirtualForceSensorParamFromProperties(m_vfs, m_robot, prop["virtual_force_sensor"], std::string(m_profile.instance_name));
    // ref force port
    int npforce = m_robot->numSensors(hrp::Sensor::FORCE);
    int nvforce = m_vfs.size();
    int nforce  = npforce + nvforce;
    m_ref_force.resize(nforce);
    m_ref_forceIn.resize(nforce);
    m_limbCOPOffset.resize(nforce);
    m_limbCOPOffsetOut.resize(nforce);
    for (unsigned int i=0; i<npforce; i++){
        sensor_names.push_back(m_robot->sensor(hrp::Sensor::FORCE, i)->name);
    }
    for (unsigned int i=0; i<nvforce; i++){
        for ( std::map<std::string, hrp::VirtualForceSensorParam>::iterator it = m_vfs.begin(); it != m_vfs.end(); it++ ) {
            if (it->second.id == i) sensor_names.push_back(it->first);
        }
    }
    // set ref force port
    std::cerr << "[" << m_profile.instance_name << "] force sensor ports (" << nforce << ")" << std::endl;
    for (unsigned int i=0; i<nforce; i++){
        m_ref_forceIn[i] = new InPort<TimedDoubleSeq>(std::string("ref_"+sensor_names[i]).c_str(), m_ref_force[i]);
        m_ref_force[i].data.length(6);
        registerInPort(std::string("ref_"+sensor_names[i]).c_str(), *m_ref_forceIn[i]);
        std::cerr << "[" << m_profile.instance_name << "]   name = " << std::string("ref_"+sensor_names[i]) << std::endl;
        ref_forces.push_back(hrp::Vector3(0,0,0));
    }
    // set limb cop offset port
    std::cerr << "[" << m_profile.instance_name << "] limbCOPOffset ports (" << nforce << ")" << std::endl;
    for (unsigned int i=0; i<nforce; i++){
        std::string nm("limbCOPOffset_"+sensor_names[i]);
        m_limbCOPOffsetOut[i] = new OutPort<TimedPoint3D>(nm.c_str(), m_limbCOPOffset[i]);
        registerOutPort(nm.c_str(), *m_limbCOPOffsetOut[i]);
        m_limbCOPOffset[i].data.x = m_limbCOPOffset[i].data.y = m_limbCOPOffset[i].data.z = 0.0;
        std::cerr << "[" << m_profile.instance_name << "]   name = " << nm << std::endl;
    }
    sbp_offset = hrp::Vector3(0,0,0);
    sbp_cog_offset = hrp::Vector3(0,0,0);
    //use_force = MODE_NO_FORCE;
    use_force = MODE_REF_FORCE;

    if (ikp.find("rleg") != ikp.end() && ikp.find("lleg") != ikp.end()) {
      is_legged_robot = true;
    } else {
      is_legged_robot = false;
    }

    m_accRef.data.ax = m_accRef.data.ay = m_accRef.data.az = 0.0;
    prev_imu_sensor_vel = hrp::Vector3::Zero();

    graspless_manip_mode = false;
    graspless_manip_arm = "arms";
    graspless_manip_p_gain = hrp::Vector3::Zero();

    is_stop_mode = false;
    has_ik_failed = false;

    pos_ik_thre = 0.1*1e-3; // [m]
    rot_ik_thre = (1e-5)*M_PI/180.0; // [rad]

    return RTC::RTC_OK;
}



RTC::ReturnCode_t AutoBalancer::onFinalize()
{
  delete zmp_interpolator;
  delete transition_interpolator;
  delete adjust_footstep_interpolator;
  return RTC::RTC_OK;
}

/*
  RTC::ReturnCode_t AutoBalancer::onStartup(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t AutoBalancer::onShutdown(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

RTC::ReturnCode_t AutoBalancer::onActivated(RTC::UniqueId ec_id)
{
    std::cerr << "[" << m_profile.instance_name<< "] onActivated(" << ec_id << ")" << std::endl;
    
    return RTC::RTC_OK;
}

RTC::ReturnCode_t AutoBalancer::onDeactivated(RTC::UniqueId ec_id)
{
  std::cerr << "[" << m_profile.instance_name<< "] onDeactivated(" << ec_id << ")" << std::endl;
  Guard guard(m_mutex);
  if (control_mode == MODE_ABC) {
    control_mode = MODE_SYNC_TO_IDLE;
    double tmp_ratio = 0.0;
    transition_interpolator->go(&tmp_ratio, m_dt, true); // sync in one controller loop
  }
  return RTC::RTC_OK;
}

#define DEBUGP ((m_debugLevel==1 && loop%200==0) || m_debugLevel > 1 )
//#define DEBUGP2 ((loop%200==0))
#define DEBUGP2 (false)
RTC::ReturnCode_t AutoBalancer::onExecute(RTC::UniqueId ec_id)
{
  // std::cerr << "AutoBalancer::onExecute(" << ec_id << ")" << std::endl;
    loop ++;
    if (m_qRefIn.isNew()) {
        m_qRefIn.read();
    }
    if (m_basePosIn.isNew()) {
      m_basePosIn.read();
      input_basePos(0) = m_basePos.data.x;
      input_basePos(1) = m_basePos.data.y;
      input_basePos(2) = m_basePos.data.z;
    }
    if (m_baseRpyIn.isNew()) {
      m_baseRpyIn.read();
      input_baseRot = hrp::rotFromRpy(m_baseRpy.data.r, m_baseRpy.data.p, m_baseRpy.data.y);
    }
    if (m_zmpIn.isNew()) {
      m_zmpIn.read();
      input_zmp(0) = m_zmp.data.x;
      input_zmp(1) = m_zmp.data.y;
      input_zmp(2) = m_zmp.data.z;
    }
    for (unsigned int i=0; i<m_ref_forceIn.size(); i++){
        if ( m_ref_forceIn[i]->isNew() ) {
            m_ref_forceIn[i]->read();
        }
    }
    if (m_optionalDataIn.isNew()) {
        m_optionalDataIn.read();
        if (is_legged_robot) {
          if (m_optionalData.data.length() >= contact_states_index_map.size()*2) {
            // current optionalData is contactstates x limb and controlSwingSupportTime x limb
            //   If contactStates in optionalData is 1.0, m_contactStates is true. Otherwise, false.
            m_contactStates.data[contact_states_index_map["rleg"]] = (std::fabs(m_optionalData.data[contact_states_index_map["rleg"]]-1.0)<0.1)?true:false;
            m_contactStates.data[contact_states_index_map["lleg"]] = (std::fabs(m_optionalData.data[contact_states_index_map["lleg"]]-1.0)<0.1)?true:false;
            m_controlSwingSupportTime.data[contact_states_index_map["rleg"]] = m_optionalData.data[contact_states_index_map["rleg"]+contact_states_index_map.size()];
            m_controlSwingSupportTime.data[contact_states_index_map["lleg"]] = m_optionalData.data[contact_states_index_map["lleg"]+contact_states_index_map.size()];
            if ( !m_contactStates.data[contact_states_index_map["rleg"]] && !m_contactStates.data[contact_states_index_map["lleg"]] ) { // If two feet have no contact, force set double support contact
              m_contactStates.data[contact_states_index_map["rleg"]] = true;
              m_contactStates.data[contact_states_index_map["lleg"]] = true;
            }
          }
        }
    }
    if (m_emergencySignalIn.isNew()){
        m_emergencySignalIn.read();
        // if (!is_stop_mode) {
        //     std::cerr << "[" << m_profile.instance_name << "] emergencySignal is set!" << std::endl;
        //     is_stop_mode = true;
        // }
    }

    Guard guard(m_mutex);
    hrp::Vector3 ref_basePos;
    hrp::Matrix33 ref_baseRot;
    hrp::Vector3 rel_ref_zmp; // ref zmp in base frame
    if ( is_legged_robot ) {
      getCurrentParameters();
      getTargetParameters();
      bool is_transition_interpolator_empty = transition_interpolator->isEmpty();
      if (!is_transition_interpolator_empty) {
        transition_interpolator->get(&transition_interpolator_ratio, true);
      } else {
        transition_interpolator_ratio = 1.0;
      }
      if (control_mode != MODE_IDLE ) {
        solveLimbIK();
        rel_ref_zmp = m_robot->rootLink()->R.transpose() * (ref_zmp - m_robot->rootLink()->p);
      } else {
        for ( std::map<std::string, ABCIKparam>::iterator it = ikp.begin(); it != ikp.end(); it++ ) {
          if (std::find(leg_names.begin(), leg_names.end(), it->first) != leg_names.end() ) {
            it->second.current_p0 = it->second.target_link->p;
            it->second.current_r0 = it->second.target_link->R;
          }
        }
        rel_ref_zmp = input_zmp;
      }
      // transition
      if (!is_transition_interpolator_empty) {
        // transition_interpolator_ratio 0=>1 : IDLE => ABC
        // transition_interpolator_ratio 1=>0 : ABC => IDLE
        ref_basePos = (1-transition_interpolator_ratio) * input_basePos + transition_interpolator_ratio * m_robot->rootLink()->p;
        rel_ref_zmp = (1-transition_interpolator_ratio) * input_zmp + transition_interpolator_ratio * rel_ref_zmp;
        rats::mid_rot(ref_baseRot, transition_interpolator_ratio, input_baseRot, m_robot->rootLink()->R);
        for ( int i = 0; i < m_robot->numJoints(); i++ ) {
          m_robot->joint(i)->q = (1-transition_interpolator_ratio) * m_qRef.data[i] + transition_interpolator_ratio * m_robot->joint(i)->q;
        }
      } else {
        ref_basePos = m_robot->rootLink()->p;
        ref_baseRot = m_robot->rootLink()->R;
      }
      // mode change for sync
      if (control_mode == MODE_SYNC_TO_ABC) {
        control_mode = MODE_ABC;
      } else if (control_mode == MODE_SYNC_TO_IDLE && transition_interpolator->isEmpty() ) {
        std::cerr << "[" << m_profile.instance_name << "] Finished cleanup" << std::endl;
        control_mode = MODE_IDLE;
      }
    }
    if ( m_qRef.data.length() != 0 ) { // initialized
      if (is_legged_robot) {
        for ( int i = 0; i < m_robot->numJoints(); i++ ){
          m_qRef.data[i] = m_robot->joint(i)->q;
        }
      }
      m_qOut.write();
    }
    if (is_legged_robot) {
      // basePos
      m_basePos.data.x = ref_basePos(0);
      m_basePos.data.y = ref_basePos(1);
      m_basePos.data.z = ref_basePos(2);
      m_basePos.tm = m_qRef.tm;
      // baseRpy
      hrp::Vector3 baseRpy = hrp::rpyFromRot(ref_baseRot);
      m_baseRpy.data.r = baseRpy(0);
      m_baseRpy.data.p = baseRpy(1);
      m_baseRpy.data.y = baseRpy(2);
      m_baseRpy.tm = m_qRef.tm;
      // baseTform
      double *tform_arr = m_baseTform.data.get_buffer();
      tform_arr[0] = m_basePos.data.x;
      tform_arr[1] = m_basePos.data.y;
      tform_arr[2] = m_basePos.data.z;
      hrp::setMatrix33ToRowMajorArray(ref_baseRot, tform_arr, 3);
      m_baseTform.tm = m_qRef.tm;
      // basePose
      m_basePose.data.position.x = m_basePos.data.x;
      m_basePose.data.position.y = m_basePos.data.y;
      m_basePose.data.position.z = m_basePos.data.z;
      m_basePose.data.orientation.r = m_baseRpy.data.r;
      m_basePose.data.orientation.p = m_baseRpy.data.p;
      m_basePose.data.orientation.y = m_baseRpy.data.y;
      m_basePose.tm = m_qRef.tm;
      // zmp
      m_zmp.data.x = rel_ref_zmp(0);
      m_zmp.data.y = rel_ref_zmp(1);
      m_zmp.data.z = rel_ref_zmp(2);
      m_zmp.tm = m_qRef.tm;
      // cog
      m_cog.data.x = ref_cog(0);
      m_cog.data.y = ref_cog(1);
      m_cog.data.z = ref_cog(2);
      m_cog.tm = m_qRef.tm;
    }
    m_basePosOut.write();
    m_baseRpyOut.write();
    m_baseTformOut.write();
    m_basePoseOut.write();
    m_zmpOut.write();
    m_cogOut.write();

    // reference acceleration
    hrp::Sensor* sen = m_robot->sensor<hrp::RateGyroSensor>("gyrometer");
    if (sen != NULL) {
      hrp::Vector3 imu_sensor_pos = sen->link->p + sen->link->R * sen->localPos;
      hrp::Vector3 imu_sensor_vel = (imu_sensor_pos - prev_imu_sensor_pos)/m_dt;
      // convert to imu sensor local acceleration
      hrp::Vector3 acc = (sen->link->R * sen->localR).transpose() * (imu_sensor_vel - prev_imu_sensor_vel)/m_dt;
      m_accRef.data.ax = acc(0); m_accRef.data.ay = acc(1); m_accRef.data.az = acc(2);
      m_accRefOut.write();
      prev_imu_sensor_pos = imu_sensor_pos;
      prev_imu_sensor_vel = imu_sensor_vel;
    }

    // control parameters
    m_contactStates.tm = m_qRef.tm;
    m_contactStatesOut.write();
    m_controlSwingSupportTime.tm = m_qRef.tm;
    m_controlSwingSupportTimeOut.write();

    for (unsigned int i=0; i<m_limbCOPOffsetOut.size(); i++){
        m_limbCOPOffset[i].tm = m_qRef.tm;
        m_limbCOPOffsetOut[i]->write();
    }

    return RTC::RTC_OK;
}

void AutoBalancer::getCurrentParameters()
{
  current_root_p = m_robot->rootLink()->p;
  current_root_R = m_robot->rootLink()->R;
  for ( int i = 0; i < m_robot->numJoints(); i++ ){
    qorg[i] = m_robot->joint(i)->q;
  }
}

void AutoBalancer::getTargetParameters()
{
  // joint angles
  for ( int i = 0; i < m_robot->numJoints(); i++ ){
    m_robot->joint(i)->q = m_qRef.data[i];
    qrefv[i] = m_robot->joint(i)->q;
  }
  // basepos, rot, zmp
  m_robot->rootLink()->p = input_basePos;
  m_robot->rootLink()->R = input_baseRot;
  m_robot->calcForwardKinematics();
  //
  if (control_mode != MODE_IDLE) {
    coordinates tmp_fix_coords;
    if (!zmp_interpolator->isEmpty()) {
      double default_zmp_offsets_output[leg_names.size() * 3];
      zmp_interpolator->get(default_zmp_offsets_output, true);
      for (size_t i = 0; i < leg_names.size(); i++)
        for (size_t j = 0; j < 3; j++)
          default_zmp_offsets[i](j) = default_zmp_offsets_output[i*3+j];
      if (DEBUGP) {
        std::cerr << "[" << m_profile.instance_name << "] default_zmp_offsets (interpolated)" << std::endl;
        for (size_t i = 0; i < leg_names.size(); i++)
            std::cerr << "[" << m_profile.instance_name << "]   " << leg_names[i] << " = " << default_zmp_offsets[i].format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "    [", "]")) << "[m]" << std::endl;
      }
    }
    if ( gg_is_walking ) {
      gg->set_default_zmp_offsets(default_zmp_offsets);
      gg_solved = gg->proc_one_tick();
      {
          std::map<leg_type, std::string> leg_type_map = gg->get_leg_type_map();
          coordinates tmpc;
          // for support leg
          for (std::vector<step_node>::const_iterator it = gg->get_support_leg_steps().begin(); it != gg->get_support_leg_steps().end(); it++) {
              coordinates sp_coords = it->worldcoords;
              coordinates(ikp[leg_type_map[it->l_r]].localPos,
                          ikp[leg_type_map[it->l_r]].localR).inverse_transformation(tmpc);
              sp_coords.transform(tmpc);
              ikp[leg_type_map[it->l_r]].target_p0 = sp_coords.pos;
              ikp[leg_type_map[it->l_r]].target_r0 = sp_coords.rot;
          }
          // for swing leg
          for (std::vector<step_node>::const_iterator it = gg->get_swing_leg_steps().begin(); it != gg->get_swing_leg_steps().end(); it++) {
              coordinates sw_coords = it->worldcoords;
              coordinates(ikp[leg_type_map[it->l_r]].localPos,
                          ikp[leg_type_map[it->l_r]].localR).inverse_transformation(tmpc);
              sw_coords.transform(tmpc);
              ikp[leg_type_map[it->l_r]].target_p0 = sw_coords.pos;
              ikp[leg_type_map[it->l_r]].target_r0 = sw_coords.rot;
          }
      }
      gg->get_swing_support_mid_coords(tmp_fix_coords);
      // TODO : assume biped
      switch (gg->get_current_support_states().front()) {
      case BOTH:
        m_contactStates.data[contact_states_index_map["rleg"]] = true;
        m_contactStates.data[contact_states_index_map["lleg"]] = true;
        break;
      case RLEG:
        m_contactStates.data[contact_states_index_map["rleg"]] = true;
        m_contactStates.data[contact_states_index_map["lleg"]] = false;
        break;
      case LLEG:
        m_contactStates.data[contact_states_index_map["rleg"]] = false;
        m_contactStates.data[contact_states_index_map["lleg"]] = true;
        break;
      default:
          if (DEBUGP) std::cerr << "not implemented yet " << std::endl;
        break;
      }
      m_controlSwingSupportTime.data[contact_states_index_map["rleg"]] = gg->get_current_swing_time(0);
      m_controlSwingSupportTime.data[contact_states_index_map["lleg"]] = gg->get_current_swing_time(1);
      m_limbCOPOffset[contact_states_index_map[gg->get_swing_leg_names().front()]].data.x = gg->get_swing_foot_zmp_offsets().front()(0);
      m_limbCOPOffset[contact_states_index_map[gg->get_swing_leg_names().front()]].data.y = gg->get_swing_foot_zmp_offsets().front()(1);
      m_limbCOPOffset[contact_states_index_map[gg->get_swing_leg_names().front()]].data.z = gg->get_swing_foot_zmp_offsets().front()(2);
      m_limbCOPOffset[contact_states_index_map[gg->get_support_leg_names().front()]].data.x = gg->get_support_foot_zmp_offsets().front()(0);
      m_limbCOPOffset[contact_states_index_map[gg->get_support_leg_names().front()]].data.y = gg->get_support_foot_zmp_offsets().front()(1);
      m_limbCOPOffset[contact_states_index_map[gg->get_support_leg_names().front()]].data.z = gg->get_support_foot_zmp_offsets().front()(2);
    } else {
      tmp_fix_coords = fix_leg_coords;
      // double support by default
      m_contactStates.data[contact_states_index_map["rleg"]] = true;
      m_contactStates.data[contact_states_index_map["lleg"]] = true;
      // controlSwingSupportTime is not used while double support period, 1.0 is neglected
      m_controlSwingSupportTime.data[contact_states_index_map["rleg"]] = 1.0;
      m_controlSwingSupportTime.data[contact_states_index_map["lleg"]] = 1.0;
      m_limbCOPOffset[contact_states_index_map["rleg"]].data.x = default_zmp_offsets[0](0);
      m_limbCOPOffset[contact_states_index_map["rleg"]].data.y = default_zmp_offsets[0](1);
      m_limbCOPOffset[contact_states_index_map["rleg"]].data.z = default_zmp_offsets[0](2);
      m_limbCOPOffset[contact_states_index_map["lleg"]].data.x = default_zmp_offsets[1](0);
      m_limbCOPOffset[contact_states_index_map["lleg"]].data.y = default_zmp_offsets[1](1);
      m_limbCOPOffset[contact_states_index_map["lleg"]].data.z = default_zmp_offsets[1](2);
    }
    if (!adjust_footstep_interpolator->isEmpty()) {
        double tmp = 0.0;
        adjust_footstep_interpolator->get(&tmp, true);
        //std::cerr << "[" << m_profile.instance_name << "] adjust ratio " << tmp << std::endl;
        ikp["rleg"].target_p0 = (1-tmp) * ikp["rleg"].adjust_interpolation_org_p0 + tmp*ikp["rleg"].adjust_interpolation_target_p0;
        ikp["lleg"].target_p0 = (1-tmp) * ikp["lleg"].adjust_interpolation_org_p0 + tmp*ikp["lleg"].adjust_interpolation_target_p0;
        rats::mid_rot(ikp["rleg"].target_r0, tmp, ikp["rleg"].adjust_interpolation_org_r0, ikp["rleg"].adjust_interpolation_target_r0);
        rats::mid_rot(ikp["lleg"].target_r0, tmp, ikp["lleg"].adjust_interpolation_org_r0, ikp["lleg"].adjust_interpolation_target_r0);
        coordinates tmprc, tmplc;
        tmprc.pos = ikp["rleg"].target_p0 + ikp["rleg"].target_r0 * ikp["rleg"].localPos;
        tmprc.rot = ikp["rleg"].target_r0 * ikp["rleg"].localR;
        tmplc.pos = ikp["lleg"].target_p0 + ikp["lleg"].target_r0 * ikp["lleg"].localPos;
        tmplc.rot = ikp["lleg"].target_r0 * ikp["lleg"].localR;
        rats::mid_coords(fix_leg_coords, 0.5, tmprc, tmplc);
        tmp_fix_coords = fix_leg_coords;
    }
    // Tempolarily modify tmp_fix_coords
    // This will be removed after seq outputs adequate waistRPY discussed in https://github.com/fkanehiro/hrpsys-base/issues/272
    {
      hrp::Vector3 ex = hrp::Vector3::UnitX();
      hrp::Vector3 ez = hrp::Vector3::UnitZ();
      hrp::Vector3 xv1 (tmp_fix_coords.rot * ex);
      xv1(2) = 0.0;
      xv1.normalize();
      hrp::Vector3 yv1(ez.cross(xv1));
      tmp_fix_coords.rot(0,0) = xv1(0); tmp_fix_coords.rot(1,0) = xv1(1); tmp_fix_coords.rot(2,0) = xv1(2);
      tmp_fix_coords.rot(0,1) = yv1(0); tmp_fix_coords.rot(1,1) = yv1(1); tmp_fix_coords.rot(2,1) = yv1(2);
      tmp_fix_coords.rot(0,2) = ez(0); tmp_fix_coords.rot(1,2) = ez(1); tmp_fix_coords.rot(2,2) = ez(2);
    }
    fixLegToCoords(tmp_fix_coords.pos, tmp_fix_coords.rot);

    /* update ref_forces ;; sp's absolute -> rmc's absolute */
    for (size_t i = 0; i < m_ref_forceIn.size(); i++) {
      hrp::Matrix33 eeR;
      hrp::Link* parentlink;
      hrp::ForceSensor* sensor = m_robot->sensor<hrp::ForceSensor>(sensor_names[i]);
      if (sensor) parentlink = sensor->link;
      else parentlink = m_vfs[sensor_names[i]].link;
      for ( std::map<std::string, ABCIKparam>::iterator it = ikp.begin(); it != ikp.end(); it++ ) {
          if (it->second.target_link->name == parentlink->name) eeR = parentlink->R * it->second.localR;
      }
      // End effector frame
      //ref_forces[i] = eeR * hrp::Vector3(m_ref_force[i].data[0], m_ref_force[i].data[1], m_ref_force[i].data[2]);
      // world frame
      ref_forces[i] = tmp_fix_coords.rot * hrp::Vector3(m_ref_force[i].data[0], m_ref_force[i].data[1], m_ref_force[i].data[2]);
    }
    sbp_offset = tmp_fix_coords.rot * hrp::Vector3(sbp_offset);

    target_root_p = m_robot->rootLink()->p;
    target_root_R = m_robot->rootLink()->R;
    for ( std::map<std::string, ABCIKparam>::iterator it = ikp.begin(); it != ikp.end(); it++ ) {
      if ( control_mode == MODE_IDLE || std::find(leg_names.begin(), leg_names.end(), it->first) == leg_names.end() ) {
        it->second.target_p0 = it->second.target_link->p;
        it->second.target_r0 = it->second.target_link->R;
      }
    }

    hrp::Vector3 tmp_foot_mid_pos(hrp::Vector3::Zero());
    for (size_t i = 0; i < leg_names.size(); i++) {
        ABCIKparam& tmpikp = ikp[leg_names[i]];
        // get target_end_coords
        tmpikp.target_end_coords.pos = tmpikp.target_p0 + tmpikp.target_r0 * tmpikp.localPos;
        tmpikp.target_end_coords.rot = tmpikp.target_r0 * tmpikp.localR;
        // for foot_mid_pos
        tmp_foot_mid_pos += tmpikp.target_link->p + tmpikp.target_link->R * tmpikp.localPos + tmpikp.target_link->R * tmpikp.localR * default_zmp_offsets[i];
    }
    tmp_foot_mid_pos *= (1.0 / leg_names.size());

    //
    {
        if ( gg_is_walking && gg->get_lcg_count() == static_cast<size_t>(gg->get_default_step_time()/(2*m_dt))-1) {
            hrp::Vector3 vel_htc(calc_vel_from_hand_error(tmp_fix_coords));
            gg->set_offset_velocity_param(vel_htc(0), vel_htc(1) ,vel_htc(2));
        }//  else {
        //     if ( gg_is_walking && gg->get_lcg_count() == static_cast<size_t>(gg->get_default_step_time()/(2*m_dt))-1) {
        //         gg->set_offset_velocity_param(0,0,0);
        //     }
        // }
    }

    //

    hrp::Vector3 tmp_ref_cog(m_robot->calcCM());
    if (gg_is_walking) {
      ref_cog = gg->get_cog();
    } else {
      ref_cog = tmp_foot_mid_pos;
    }
    ref_cog(2) = tmp_ref_cog(2);
    if (gg_is_walking) {
      ref_zmp = gg->get_refzmp();
    } else {
      ref_zmp(0) = ref_cog(0);
      ref_zmp(1) = ref_cog(1);
      ref_zmp(2) = tmp_foot_mid_pos(2);
    }
  }
  // Just for ik initial value
  if (control_mode == MODE_SYNC_TO_ABC) {
    current_root_p = target_root_p;
    current_root_R = target_root_R;
    for ( std::map<std::string, ABCIKparam>::iterator it = ikp.begin(); it != ikp.end(); it++ ) {
      if ( std::find(leg_names.begin(), leg_names.end(), it->first) != leg_names.end() ) {
        it->second.target_p0 = it->second.target_link->p;
        it->second.target_r0 = it->second.target_link->R;
      }
    }
  }
};

hrp::Matrix33 AutoBalancer::OrientRotationMatrix (const hrp::Matrix33& rot, const hrp::Vector3& axis1, const hrp::Vector3& axis2)
{
  hrp::Vector3 vv = axis1.cross(axis2);
  if (fabs(vv.norm()-0.0) < 1e-5) {
    return rot;
  } else {
    Eigen::AngleAxis<double> tmpr(std::asin(vv.norm()), vv.normalized());
    return tmpr.toRotationMatrix() * rot;
  }
}

void AutoBalancer::fixLegToCoords (const hrp::Vector3& fix_pos, const hrp::Matrix33& fix_rot)
{
  // get current foot mid pos + rot
  std::vector<coordinates> foot_coords;
  for (size_t i = 0; i < leg_names.size(); i++) {
      ABCIKparam& tmpikp = ikp[leg_names[i]];
      foot_coords.push_back(coordinates((tmpikp.target_link->p + tmpikp.target_link->R * tmpikp.localPos),
                                        (tmpikp.target_link->R * tmpikp.localR)));
  }
  coordinates current_foot_mid_coords;
  multi_mid_coords(current_foot_mid_coords, foot_coords);
  hrp::Vector3 current_foot_mid_pos = current_foot_mid_coords.pos;
  hrp::Matrix33 current_foot_mid_rot = current_foot_mid_coords.rot;
  // fix root pos + rot to fix "coords" = "current_foot_mid_xx"
  hrp::Matrix33 tmpR (fix_rot * current_foot_mid_rot.transpose());
  m_robot->rootLink()->p = fix_pos + tmpR * (m_robot->rootLink()->p - current_foot_mid_pos);
  rats::rotm3times(m_robot->rootLink()->R, tmpR, m_robot->rootLink()->R);
  m_robot->calcForwardKinematics();
}

bool AutoBalancer::solveLimbIKforLimb (ABCIKparam& param)
{
  param.current_p0 = param.target_link->p;
  param.current_r0 = param.target_link->R;

  hrp::Vector3 vel_p, vel_r;
  vel_p = param.target_p0 - param.current_p0;
  rats::difference_rotation(vel_r, param.current_r0, param.target_r0);
  vel_p *= transition_interpolator_ratio;
  vel_r *= transition_interpolator_ratio;
  param.manip->calcInverseKinematics2Loop(vel_p, vel_r, 1.0, 0.001, 0.01, &qrefv);
  // IK check
  vel_p = param.target_p0 - param.target_link->p;
  rats::difference_rotation(vel_r, param.current_r0, param.target_link->R);
  if (vel_p.norm() > pos_ik_thre && transition_interpolator->isEmpty()) {
      std::cerr << "[" << m_profile.instance_name << "] Too large IK error (vel_p) = [" << vel_p(0) << " " << vel_p(1) << " " << vel_p(2) << "][m]" << std::endl;
      has_ik_failed = true;
  }
  if (vel_r.norm() > rot_ik_thre && transition_interpolator->isEmpty()) {
      std::cerr << "[" << m_profile.instance_name << "] Too large IK error (vel_r) = [" << vel_r(0) << " " << vel_r(1) << " " << vel_r(2) << "][rad]" << std::endl;
      has_ik_failed = true;
  }
  return true;
}

void AutoBalancer::solveLimbIK ()
{
  for ( std::map<std::string, ABCIKparam>::iterator it = ikp.begin(); it != ikp.end(); it++ ) {
    if (it->second.is_active) {
      for ( int j = 0; j < it->second.manip->numJoints(); j++ ){
	int i = it->second.manip->joint(j)->jointId;
	m_robot->joint(i)->q = qorg[i];
      }
    }
  }
  m_robot->rootLink()->p = current_root_p;
  m_robot->rootLink()->R = current_root_R;
  m_robot->calcForwardKinematics();
  hrp::Vector3 tmp_input_sbp = hrp::Vector3(0,0,0);
  static_balance_point_proc_one(tmp_input_sbp, ref_zmp(2));
  hrp::Vector3 dif_cog = tmp_input_sbp - ref_cog;
  dif_cog(2) = m_robot->rootLink()->p(2) - target_root_p(2);
  m_robot->rootLink()->p = m_robot->rootLink()->p + -1 * move_base_gain * dif_cog;
  m_robot->rootLink()->R = target_root_R;
  // Fix for toe joint
  for ( std::map<std::string, ABCIKparam>::iterator it = ikp.begin(); it != ikp.end(); it++ ) {
      if (it->second.is_active && (it->first.find("leg") != std::string::npos) && it->second.manip->numJoints() == 7) {
          int i = it->second.target_link->jointId;
          if (gg->get_swing_leg_names().front() == it->first) {
              m_robot->joint(i)->q = qrefv[i] + -1 * gg->get_foot_dif_rot_angle();
          } else {
              m_robot->joint(i)->q = qrefv[i];
          }
      }
  }
  m_robot->calcForwardKinematics();

  for ( std::map<std::string, ABCIKparam>::iterator it = ikp.begin(); it != ikp.end(); it++ ) {
    if (it->second.is_active) solveLimbIKforLimb(it->second);
  }
  if (gg_is_walking && !gg_solved) stopWalking ();
}

/*
  RTC::ReturnCode_t AutoBalancer::onAborting(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t AutoBalancer::onError(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t AutoBalancer::onReset(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t AutoBalancer::onStateUpdate(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t AutoBalancer::onRateChanged(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

void AutoBalancer::startABCparam(const OpenHRP::AutoBalancerService::StrSequence& limbs)
{
  std::cerr << "[" << m_profile.instance_name << "] start auto balancer mode" << std::endl;
  Guard guard(m_mutex);
  double tmp_ratio = 0.0;
  transition_interpolator->clear();
  transition_interpolator->set(&tmp_ratio);
  tmp_ratio = 1.0;
  transition_interpolator->go(&tmp_ratio, transition_time, true);
  for ( std::map<std::string, ABCIKparam>::iterator it = ikp.begin(); it != ikp.end(); it++ ) {
    it->second.is_active = false;
  }

  for (size_t i = 0; i < limbs.length(); i++) {
    ABCIKparam& tmp = ikp[std::string(limbs[i])];
    tmp.is_active = true;
    std::cerr << "[" << m_profile.instance_name << "]   limb [" << std::string(limbs[i]) << "]" << std::endl;
  }

  control_mode = MODE_SYNC_TO_ABC;
}

void AutoBalancer::stopABCparam()
{
  std::cerr << "[" << m_profile.instance_name << "] stop auto balancer mode" << std::endl;
  //Guard guard(m_mutex);
  double tmp_ratio = 1.0;
  transition_interpolator->clear();
  transition_interpolator->set(&tmp_ratio);
  tmp_ratio = 0.0;
  transition_interpolator->go(&tmp_ratio, transition_time, true);
  control_mode = MODE_SYNC_TO_IDLE;
}

void AutoBalancer::startWalking ()
{
  if ( control_mode != MODE_ABC ) {
    return_control_mode = control_mode;
    OpenHRP::AutoBalancerService::StrSequence fix_limbs;
    fix_limbs.length(2);
    fix_limbs[0] = "rleg";
    fix_limbs[1] = "lleg";
    startABCparam(fix_limbs);
    waitABCTransition();
  }
  {
    Guard guard(m_mutex);
    has_ik_failed = false;
    std::vector<std::string> init_swing_leg_names(gg->get_footstep_front_leg_names());
    std::vector<std::string> tmp_all_limbs(leg_names);
    std::vector<std::string> init_support_leg_names;
    std::sort(tmp_all_limbs.begin(), tmp_all_limbs.end());
    std::sort(init_swing_leg_names.begin(), init_swing_leg_names.end());
    std::set_difference(tmp_all_limbs.begin(), tmp_all_limbs.end(),
                        init_swing_leg_names.begin(), init_swing_leg_names.end(),
                        std::back_inserter(init_support_leg_names));
    std::vector<step_node> init_support_leg_steps, init_swing_leg_dst_steps;
    for (std::vector<std::string>::iterator it = init_support_leg_names.begin(); it != init_support_leg_names.end(); it++)
        init_support_leg_steps.push_back(step_node(*it, ikp[*it].target_end_coords, 0, 0, 0, 0));
    for (std::vector<std::string>::iterator it = init_swing_leg_names.begin(); it != init_swing_leg_names.end(); it++)
        init_swing_leg_dst_steps.push_back(step_node(*it, ikp[*it].target_end_coords, 0, 0, 0, 0));
    gg->set_default_zmp_offsets(default_zmp_offsets);
    gg->initialize_gait_parameter(ref_cog, init_support_leg_steps, init_swing_leg_dst_steps);
  }
  while ( !gg->proc_one_tick() );
  {
    Guard guard(m_mutex);
    gg_is_walking = gg_solved = true;
  }
}

void AutoBalancer::stopWalking ()
{
  std::vector<coordinates> tmp_end_coords_list;
  for (std::vector<string>::iterator it = leg_names.begin(); it != leg_names.end(); it++) {
      tmp_end_coords_list.push_back(ikp[*it].target_end_coords);
  }
  multi_mid_coords(fix_leg_coords, tmp_end_coords_list);
  fixLegToCoords(fix_leg_coords.pos, fix_leg_coords.rot);
  gg->clear_footstep_nodes_list();
  if (return_control_mode == MODE_IDLE) stopABCparam();
  gg_is_walking = false;
}

bool AutoBalancer::startAutoBalancer (const OpenHRP::AutoBalancerService::StrSequence& limbs)
{
  if (control_mode == MODE_IDLE) {
    has_ik_failed = false;
    startABCparam(limbs);
    waitABCTransition();
    return_control_mode = MODE_ABC;
    return true;
  } else {
    return false;
  }
}

bool AutoBalancer::stopAutoBalancer ()
{
  if (control_mode == MODE_ABC) {
    stopABCparam();
    waitABCTransition();
    return true;
  } else {
    return false;
  }
}

void AutoBalancer::waitABCTransition()
{
  while (!transition_interpolator->isEmpty()) usleep(1000);
  usleep(1000);
}
bool AutoBalancer::goPos(const double& x, const double& y, const double& th)
{
  if ( !gg_is_walking && !is_stop_mode) {
    gg->set_all_limbs(leg_names);
    coordinates start_ref_coords;
    mid_coords(start_ref_coords, 0.5, ikp["rleg"].target_end_coords, ikp["lleg"].target_end_coords);
    gg->go_pos_param_2_footstep_nodes_list(x, y, th,
                                           (y > 0 ? boost::assign::list_of(ikp["rleg"].target_end_coords) : boost::assign::list_of(ikp["lleg"].target_end_coords)),
                                           start_ref_coords,
                                           (y > 0 ? boost::assign::list_of(RLEG) : boost::assign::list_of(LLEG)));
    startWalking();
    return true;
  } else {
    std::cerr << "[" << m_profile.instance_name << "] Cannot goPos while walking." << std::endl;
    return false;
  }
}

bool AutoBalancer::goVelocity(const double& vx, const double& vy, const double& vth)
{
  if (gg_is_walking && gg_solved) {
    gg->set_velocity_param(vx, vy, vth);
  } else {
    coordinates ref_coords;
    mid_coords(ref_coords, 0.5, ikp["rleg"].target_end_coords, ikp["lleg"].target_end_coords);
    gg->initialize_velocity_mode(ref_coords, vx, vy, vth);
    startWalking();
  }
  return true;
}

bool AutoBalancer::goStop ()
{
  gg->finalize_velocity_mode();
  waitFootSteps();
  return true;
}

bool AutoBalancer::emergencyStop ()
{
  std::cerr << "[" << m_profile.instance_name << "] emergencyStop" << std::endl;
  // is_stop_mode = true;
  gg->emergency_stop();
  waitFootSteps();
  return true;
}

bool AutoBalancer::releaseEmergencyStop ()
{
  if (is_stop_mode) {
      std::cerr << "[" << m_profile.instance_name << "] releaseEmergencyStop" << std::endl;
      is_stop_mode = false;
  }
  return true;
}

bool AutoBalancer::setFootSteps(const OpenHRP::AutoBalancerService::FootstepsSequence& fss, CORBA::Long overwrite_fs_idx)
{
  OpenHRP::AutoBalancerService::StepParamsSequence spss;
  spss.length(fss.length());
  // If gg_is_walking is false, initial footstep will be double support. So, set 0 for step_height and toe heel angles.
  // If gg_is_walking is true, do not set to 0.
  for (size_t i = 0; i < spss.length(); i++) {
      spss[i].sps.length(fss[i].fs.length());
      for (size_t j = 0; j < spss[i].sps.length(); j++) {
          spss[i].sps[j].step_height = ((!gg_is_walking && i==0) ? 0.0 : gg->get_default_step_height());
          spss[i].sps[j].step_time = gg->get_default_step_time();
          spss[i].sps[j].toe_angle = ((!gg_is_walking && i==0) ? 0.0 : gg->get_default_step_height());
          spss[i].sps[j].heel_angle = ((!gg_is_walking && i==0) ? 0.0 : gg->get_default_step_height());
      }
  }
  setFootStepsWithParam(fss, spss, overwrite_fs_idx);
}

bool AutoBalancer::setFootStepsWithParam(const OpenHRP::AutoBalancerService::FootstepsSequence& fss, const OpenHRP::AutoBalancerService::StepParamsSequence& spss, CORBA::Long overwrite_fs_idx)
{
    if (!is_stop_mode) {
        std::cerr << "[" << m_profile.instance_name << "] setFootStepsList" << std::endl;

        // Initial footstep Snapping
        coordinates tmpfs, fstrans;
        step_node initial_support_step, initial_input_step;
        {
            std::vector<step_node> initial_support_steps;
            if (gg_is_walking) {
                if (overwrite_fs_idx <= 0) {
                    std::cerr << "[" << m_profile.instance_name << "]   Invalid overwrite index = " << overwrite_fs_idx << std::endl;
                    return false;
                }
                if (!gg->get_footstep_nodes_by_index(initial_support_steps, overwrite_fs_idx-1)) {
                    std::cerr << "[" << m_profile.instance_name << "]   Invalid overwrite index = " << overwrite_fs_idx << std::endl;
                    return false;
                }
            } else {
                // If walking, snap initial leg to current ABC foot coords.
                for (size_t i = 0; i < fss[0].fs.length(); i++) {
                    initial_support_steps.push_back(step_node(std::string(fss[0].fs[i].leg), ikp[std::string(fss[0].fs[i].leg)].target_end_coords, 0, 0, 0, 0));
                }
            }
            initial_support_step = initial_support_steps.front(); /* use only one leg for representation */
        }
        {
            std::map<leg_type, std::string> leg_type_map = gg->get_leg_type_map();
            for (size_t i = 0; i < fss[0].fs.length(); i++) {
                if (std::string(fss[0].fs[i].leg) == leg_type_map[initial_support_step.l_r]) {
                    coordinates tmp;
                    memcpy(tmp.pos.data(), fss[0].fs[i].pos, sizeof(double)*3);
                    tmp.rot = (Eigen::Quaternion<double>(fss[0].fs[i].rot[0], fss[0].fs[i].rot[1], fss[0].fs[i].rot[2], fss[0].fs[i].rot[3])).normalized().toRotationMatrix(); // rtc: (x, y, z, w) but eigen: (w, x, y, z)
                    initial_input_step = step_node(std::string(fss[0].fs[i].leg), tmp, 0, 0, 0, 0);
                }
            }
        }

        // Get footsteps
        std::vector< std::vector<coordinates> > fs_vec_list;
        std::vector< std::vector<std::string> > leg_name_vec_list;
        for (size_t i = 0; i < fss.length(); i++) {
            std::vector<coordinates> fs_vec;
            std::vector<std::string> leg_name_vec;
            for (size_t j = 0; j < fss[i].fs.length(); j++) {
                std::string leg(fss[i].fs[j].leg);
                if (std::find(leg_names.begin(), leg_names.end(), leg) != leg_names.end()) {
                    memcpy(tmpfs.pos.data(), fss[i].fs[j].pos, sizeof(double)*3);
                    tmpfs.rot = (Eigen::Quaternion<double>(fss[i].fs[j].rot[0], fss[i].fs[j].rot[1], fss[i].fs[j].rot[2], fss[i].fs[j].rot[3])).normalized().toRotationMatrix(); // rtc: (x, y, z, w) but eigen: (w, x, y, z)
                    initial_input_step.worldcoords.transformation(fstrans, tmpfs);
                    tmpfs = initial_support_step.worldcoords;
                    tmpfs.transform(fstrans);
                } else {
                    std::cerr << "[" << m_profile.instance_name << "]   No such target : " << leg << std::endl;
                    return false;
                }
                leg_name_vec.push_back(leg);
                fs_vec.push_back(tmpfs);
            }
            leg_name_vec_list.push_back(leg_name_vec);
            fs_vec_list.push_back(fs_vec);
        }
        if (spss.length() != fs_vec_list.size()) {
            std::cerr << "[" << m_profile.instance_name << "]   StepParam length " << spss.length () << " != Footstep length " << fs_vec_list.size() << std::endl;
            return false;
        }
        std::cerr << "[" << m_profile.instance_name << "] print footsteps " << std::endl;
        std::vector< std::vector<step_node> > fnsl;
        for (size_t i = 0; i < fs_vec_list.size(); i++) {
            if (!(gg_is_walking && i == 0)) { // If initial footstep, e.g., not walking, pass user-defined footstep list. If walking, pass cdr footsteps in order to neglect initial double support leg.
                std::vector<step_node> tmp_fns;
                for (size_t j = 0; j < fs_vec_list.at(j).size(); j++) {
                    tmp_fns.push_back(step_node(leg_name_vec_list[i][j], fs_vec_list[i][j], spss[i].sps[j].step_height, spss[i].sps[j].step_time, spss[i].sps[j].toe_angle, spss[i].sps[j].heel_angle));
                }
                fnsl.push_back(tmp_fns);
            }
        }
        if (gg_is_walking) {
            std::cerr << "[" << m_profile.instance_name << "]  Set overwrite footsteps" << std::endl;
            gg->set_overwrite_foot_steps_list(fnsl);
            gg->set_overwrite_foot_step_index(overwrite_fs_idx);
        } else {
            std::cerr << "[" << m_profile.instance_name << "]  Set normal footsteps" << std::endl;
            gg->set_foot_steps_list(fnsl);
            startWalking();
        }
        return true;
    } else {
        std::cerr << "[" << m_profile.instance_name << "] Cannot setFootSteps while walking." << std::endl;
        return false;
    }
}

void AutoBalancer::waitFootSteps()
{
  //while (gg_is_walking) usleep(10);
  while (gg_is_walking || !transition_interpolator->isEmpty() )
    usleep(1000);
  usleep(1000);
  gg->set_offset_velocity_param(0,0,0);
}

void AutoBalancer::waitFootStepsEarly(const double tm)
{
  if (!gg_is_walking) { return;}
  while ( !gg->is_finalizing(tm)|| !transition_interpolator->isEmpty() )
    usleep(1000);
  usleep(1000);
  gg->set_offset_velocity_param(0,0,0);
}

bool AutoBalancer::setGaitGeneratorParam(const OpenHRP::AutoBalancerService::GaitGeneratorParam& i_param)
{
  std::cerr << "[" << m_profile.instance_name << "] setGaitGeneratorParam" << std::endl;
  gg->set_stride_parameters(i_param.stride_parameter[0], i_param.stride_parameter[1], i_param.stride_parameter[2], i_param.stride_parameter[3]);
  std::vector<hrp::Vector3> off;
  for (size_t i = 0; i < i_param.leg_default_translate_pos.length(); i++) {
      off.push_back(hrp::Vector3(i_param.leg_default_translate_pos[i][0], i_param.leg_default_translate_pos[i][1], i_param.leg_default_translate_pos[i][2]));
  }
  gg->set_leg_default_translate_pos(off);
  gg->set_default_step_time(i_param.default_step_time);
  gg->set_default_step_height(i_param.default_step_height);
  gg->set_default_double_support_ratio(i_param.default_double_support_ratio);
  gg->set_default_double_support_static_ratio(i_param.default_double_support_static_ratio);
  if (i_param.default_orbit_type == OpenHRP::AutoBalancerService::SHUFFLING) {
    gg->set_default_orbit_type(SHUFFLING);
  } else if (i_param.default_orbit_type == OpenHRP::AutoBalancerService::CYCLOID) {
    gg->set_default_orbit_type(CYCLOID);
  } else if (i_param.default_orbit_type == OpenHRP::AutoBalancerService::RECTANGLE) {
    gg->set_default_orbit_type(RECTANGLE);
  } else if (i_param.default_orbit_type == OpenHRP::AutoBalancerService::STAIR) {
    gg->set_default_orbit_type(STAIR);
  } else if (i_param.default_orbit_type == OpenHRP::AutoBalancerService::CYCLOIDDELAY) {
    gg->set_default_orbit_type(CYCLOIDDELAY);
  } else if (i_param.default_orbit_type == OpenHRP::AutoBalancerService::CYCLOIDDELAYKICK) {
    gg->set_default_orbit_type(CYCLOIDDELAYKICK);
  }
  gg->set_swing_trajectory_delay_time_offset(i_param.swing_trajectory_delay_time_offset);
  gg->set_swing_trajectory_final_distance_weight(i_param.swing_trajectory_final_distance_weight);
  gg->set_stair_trajectory_way_point_offset(hrp::Vector3(i_param.stair_trajectory_way_point_offset[0], i_param.stair_trajectory_way_point_offset[1], i_param.stair_trajectory_way_point_offset[2]));
  gg->set_cycloid_delay_kick_point_offset(hrp::Vector3(i_param.cycloid_delay_kick_point_offset[0], i_param.cycloid_delay_kick_point_offset[1], i_param.cycloid_delay_kick_point_offset[2]));  
  gg->set_gravitational_acceleration(i_param.gravitational_acceleration);
  gg->set_toe_angle(i_param.toe_angle);
  gg->set_heel_angle(i_param.heel_angle);
  gg->set_toe_pos_offset_x(i_param.toe_pos_offset_x);
  gg->set_heel_pos_offset_x(i_param.heel_pos_offset_x);
  gg->set_toe_zmp_offset_x(i_param.toe_zmp_offset_x);
  gg->set_heel_zmp_offset_x(i_param.heel_zmp_offset_x);
  std::vector<double> tmp_ratio(i_param.toe_heel_phase_ratio.get_buffer(), i_param.toe_heel_phase_ratio.get_buffer()+i_param.toe_heel_phase_ratio.length());
  std::cerr << "[" << m_profile.instance_name << "]   "; // for set_toe_heel_phase_ratio
  gg->set_toe_heel_phase_ratio(tmp_ratio);
  gg->set_use_toe_joint(i_param.use_toe_joint);
  gg->set_use_toe_heel_transition(i_param.use_toe_heel_transition);
  gg->set_optional_go_pos_finalize_footstep_num(i_param.optional_go_pos_finalize_footstep_num);

  // print
  gg->print_param(std::string(m_profile.instance_name));
  return true;
};

bool AutoBalancer::getGaitGeneratorParam(OpenHRP::AutoBalancerService::GaitGeneratorParam& i_param)
{
  gg->get_stride_parameters(i_param.stride_parameter[0], i_param.stride_parameter[1], i_param.stride_parameter[2], i_param.stride_parameter[3]);
  std::vector<hrp::Vector3> off;
  gg->get_leg_default_translate_pos(off);
  i_param.leg_default_translate_pos.length(off.size());
  for (size_t i = 0; i < i_param.leg_default_translate_pos.length(); i++) {
      i_param.leg_default_translate_pos[i].length(3);
      i_param.leg_default_translate_pos[i][0] = off[i](0);
      i_param.leg_default_translate_pos[i][1] = off[i](1);
      i_param.leg_default_translate_pos[i][2] = off[i](2);
  }
  i_param.default_step_time = gg->get_default_step_time();
  i_param.default_step_height = gg->get_default_step_height();
  i_param.default_double_support_ratio = gg->get_default_double_support_ratio();
  i_param.default_double_support_static_ratio = gg->get_default_double_support_static_ratio();
  if (gg->get_default_orbit_type() == SHUFFLING) {
    i_param.default_orbit_type = OpenHRP::AutoBalancerService::SHUFFLING;
  } else if (gg->get_default_orbit_type() == CYCLOID) {
    i_param.default_orbit_type = OpenHRP::AutoBalancerService::CYCLOID;
  } else if (gg->get_default_orbit_type() == RECTANGLE) {
    i_param.default_orbit_type = OpenHRP::AutoBalancerService::RECTANGLE;
  } else if (gg->get_default_orbit_type() == STAIR) {
    i_param.default_orbit_type = OpenHRP::AutoBalancerService::STAIR;
  } else if (gg->get_default_orbit_type() == CYCLOIDDELAY) {
    i_param.default_orbit_type = OpenHRP::AutoBalancerService::CYCLOIDDELAY;
  } else if (gg->get_default_orbit_type() == CYCLOIDDELAYKICK) {
    i_param.default_orbit_type = OpenHRP::AutoBalancerService::CYCLOIDDELAYKICK;
  }

  hrp::Vector3 tmpv = gg->get_stair_trajectory_way_point_offset();
  for (size_t i = 0; i < 3; i++) i_param.stair_trajectory_way_point_offset[i] = tmpv(i);
  tmpv = gg->get_cycloid_delay_kick_point_offset();
  for (size_t i = 0; i < 3; i++) i_param.cycloid_delay_kick_point_offset[i] = tmpv(i);
  i_param.swing_trajectory_delay_time_offset = gg->get_swing_trajectory_delay_time_offset();
  i_param.swing_trajectory_final_distance_weight = gg->get_swing_trajectory_final_distance_weight();
  i_param.gravitational_acceleration = gg->get_gravitational_acceleration();
  i_param.toe_angle = gg->get_toe_angle();
  i_param.heel_angle = gg->get_heel_angle();
  i_param.toe_pos_offset_x = gg->get_toe_pos_offset_x();
  i_param.heel_pos_offset_x = gg->get_heel_pos_offset_x();
  i_param.toe_zmp_offset_x = gg->get_toe_zmp_offset_x();
  i_param.heel_zmp_offset_x = gg->get_heel_zmp_offset_x();
  std::vector<double> ratio(gg->get_NUM_TH_PHASES(),0.0);
  gg->get_toe_heel_phase_ratio(ratio);
  for (int i = 0; i < gg->get_NUM_TH_PHASES(); i++) i_param.toe_heel_phase_ratio[i] = ratio[i];
  i_param.use_toe_joint = gg->get_use_toe_joint();
  i_param.use_toe_heel_transition = gg->get_use_toe_heel_transition();
  i_param.optional_go_pos_finalize_footstep_num = gg->get_optional_go_pos_finalize_footstep_num();
  return true;
};

bool AutoBalancer::setAutoBalancerParam(const OpenHRP::AutoBalancerService::AutoBalancerParam& i_param)
{
  std::cerr << "[" << m_profile.instance_name << "] setAutoBalancerParam" << std::endl;
  double default_zmp_offsets_array[6];
  move_base_gain = i_param.move_base_gain;
  for (size_t i = 0; i < 2; i++)
    for (size_t j = 0; j < 3; j++)
      default_zmp_offsets_array[i*3+j] = i_param.default_zmp_offsets[i][j];
  zmp_transition_time = i_param.zmp_transition_time;
  adjust_footstep_transition_time = i_param.adjust_footstep_transition_time;
  if (zmp_interpolator->isEmpty()) {
      zmp_interpolator->clear();
      zmp_interpolator->go(default_zmp_offsets_array, zmp_transition_time, true);
  } else {
      std::cerr << "[" << m_profile.instance_name << "]   default_zmp_offsets cannot be set because interpolating." << std::endl;
  }
  graspless_manip_mode = i_param.graspless_manip_mode;
  graspless_manip_arm = std::string(i_param.graspless_manip_arm);
  for (size_t j = 0; j < 3; j++)
      graspless_manip_p_gain[j] = i_param.graspless_manip_p_gain[j];
  for (size_t j = 0; j < 3; j++)
      graspless_manip_reference_trans_coords.pos[j] = i_param.graspless_manip_reference_trans_pos[j];
  graspless_manip_reference_trans_coords.rot = (Eigen::Quaternion<double>(i_param.graspless_manip_reference_trans_rot[0],
                                                                          i_param.graspless_manip_reference_trans_rot[1],
                                                                          i_param.graspless_manip_reference_trans_rot[2],
                                                                          i_param.graspless_manip_reference_trans_rot[3]).normalized().toRotationMatrix()); // rtc: (x, y, z, w) but eigen: (w, x, y, z)
  transition_time = i_param.transition_time;
  leg_names.clear();
  for (size_t i = 0; i < i_param.leg_names.length(); i++) {
      leg_names.push_back(std::string(i_param.leg_names[i]));
  }
  pos_ik_thre = i_param.pos_ik_thre;
  rot_ik_thre = i_param.rot_ik_thre;
  std::cerr << "[" << m_profile.instance_name << "]   move_base_gain = " << move_base_gain << std::endl;
  std::cerr << "[" << m_profile.instance_name << "]   default_zmp_offsets = "
            << default_zmp_offsets_array[0] << " " << default_zmp_offsets_array[1] << " " << default_zmp_offsets_array[2] << " "
            << default_zmp_offsets_array[3] << " " << default_zmp_offsets_array[4] << " " << default_zmp_offsets_array[5] << std::endl;
  std::cerr << "[" << m_profile.instance_name << "]   graspless_manip_mode = " << graspless_manip_mode << std::endl;
  std::cerr << "[" << m_profile.instance_name << "]   graspless_manip_arm = " << graspless_manip_arm << std::endl;
  std::cerr << "[" << m_profile.instance_name << "]   graspless_manip_p_gain = " << graspless_manip_p_gain.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "    [", "]")) << std::endl;
  std::cerr << "[" << m_profile.instance_name << "]   graspless_manip_reference_trans_pos = " << graspless_manip_reference_trans_coords.pos.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "    [", "]")) << std::endl;
  std::cerr << "[" << m_profile.instance_name << "]   graspless_manip_reference_trans_rot = " << graspless_manip_reference_trans_coords.rot.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", "\n", "    [", "]")) << std::endl;
  std::cerr << "[" << m_profile.instance_name << "]   transition_time = " << transition_time << "[s], zmp_transition_time = " << zmp_transition_time << "[s], adjust_footstep_transition_time = " << adjust_footstep_transition_time << "[s]" << std::endl;
  for (std::vector<std::string>::iterator it = leg_names.begin(); it != leg_names.end(); it++) std::cerr << "[" << m_profile.instance_name << "]   leg_names [" << *it << "]" << std::endl;
  std::cerr << "[" << m_profile.instance_name << "]   pos_ik_thre = " << pos_ik_thre << "[m], rot_ik_thre = " << rot_ik_thre << "[rad]" << std::endl;
  return true;
};

bool AutoBalancer::getAutoBalancerParam(OpenHRP::AutoBalancerService::AutoBalancerParam& i_param)
{
  i_param.move_base_gain = move_base_gain;
  for (size_t i = 0; i < 2; i++)
    for (size_t j = 0; j < 3; j++)
      i_param.default_zmp_offsets[i][j] = default_zmp_offsets[i](j);
  switch(control_mode) {
  case MODE_IDLE: i_param.controller_mode = OpenHRP::AutoBalancerService::MODE_IDLE; break;
  case MODE_ABC: i_param.controller_mode = OpenHRP::AutoBalancerService::MODE_ABC; break;
  case MODE_SYNC_TO_IDLE: i_param.controller_mode = OpenHRP::AutoBalancerService::MODE_SYNC_TO_IDLE; break;
  case MODE_SYNC_TO_ABC: i_param.controller_mode = OpenHRP::AutoBalancerService::MODE_SYNC_TO_ABC; break;
  default: break;
  }
  i_param.graspless_manip_mode = graspless_manip_mode;
  i_param.graspless_manip_arm = graspless_manip_arm.c_str();
  for (size_t j = 0; j < 3; j++)
      i_param.graspless_manip_p_gain[j] = graspless_manip_p_gain[j];
  for (size_t j = 0; j < 3; j++)
      i_param.graspless_manip_reference_trans_pos[j] = graspless_manip_reference_trans_coords.pos[j];
  Eigen::Quaternion<double> qt(graspless_manip_reference_trans_coords.rot);
  i_param.graspless_manip_reference_trans_rot[0] = qt.w();
  i_param.graspless_manip_reference_trans_rot[1] = qt.x();
  i_param.graspless_manip_reference_trans_rot[2] = qt.y();
  i_param.graspless_manip_reference_trans_rot[3] = qt.z();
  i_param.transition_time = transition_time;
  i_param.zmp_transition_time = zmp_transition_time;
  i_param.adjust_footstep_transition_time = adjust_footstep_transition_time;
  i_param.leg_names.length(leg_names.size());
  for (size_t i = 0; i < leg_names.size(); i++) i_param.leg_names[i] = leg_names.at(i).c_str();
  i_param.pos_ik_thre = pos_ik_thre;
  i_param.rot_ik_thre = rot_ik_thre;
  return true;
};

void AutoBalancer::copyRatscoords2Footstep(OpenHRP::AutoBalancerService::Footstep& out_fs, const rats::coordinates& in_fs)
{
  memcpy(out_fs.pos, in_fs.pos.data(), sizeof(double)*3);
  Eigen::Quaternion<double> qt(in_fs.rot);
  out_fs.rot[0] = qt.w();
  out_fs.rot[1] = qt.x();
  out_fs.rot[2] = qt.y();
  out_fs.rot[3] = qt.z();
};

bool AutoBalancer::getFootstepParam(OpenHRP::AutoBalancerService::FootstepParam& i_param)
{
  std::vector<rats::coordinates> leg_coords;
  for (size_t i = 0; i < leg_names.size(); i++) {
      ABCIKparam& tmpikp = ikp[leg_names[i]];
      leg_coords.push_back(coordinates(tmpikp.current_p0 + tmpikp.current_r0 * tmpikp.localPos,
                                       tmpikp.current_r0 * tmpikp.localR));
  }
  copyRatscoords2Footstep(i_param.rleg_coords, leg_coords[0]);
  copyRatscoords2Footstep(i_param.lleg_coords, leg_coords[1]);
  copyRatscoords2Footstep(i_param.support_leg_coords, gg->get_support_leg_steps().front().worldcoords);
  copyRatscoords2Footstep(i_param.swing_leg_coords, gg->get_swing_leg_steps().front().worldcoords);
  copyRatscoords2Footstep(i_param.swing_leg_src_coords, gg->get_swing_leg_src_steps().front().worldcoords);
  copyRatscoords2Footstep(i_param.swing_leg_dst_coords, gg->get_swing_leg_dst_steps().front().worldcoords);
  copyRatscoords2Footstep(i_param.dst_foot_midcoords, gg->get_dst_foot_midcoords());
  if (gg->get_support_leg_names().front() == "rleg") {
    i_param.support_leg = OpenHRP::AutoBalancerService::RLEG;
  } else {
    i_param.support_leg = OpenHRP::AutoBalancerService::LLEG;
  }
  switch ( gg->get_current_support_states().front() ) {
  case BOTH: i_param.support_leg_with_both = OpenHRP::AutoBalancerService::BOTH; break;
  case RLEG: i_param.support_leg_with_both = OpenHRP::AutoBalancerService::RLEG; break;
  case LLEG: i_param.support_leg_with_both = OpenHRP::AutoBalancerService::LLEG; break;
  default: break;
  }
  return true;
};

bool AutoBalancer::adjustFootSteps(const OpenHRP::AutoBalancerService::Footstep& rfootstep, const OpenHRP::AutoBalancerService::Footstep& lfootstep)
{
  std::cerr << "[" << m_profile.instance_name << "] adjustFootSteps" << std::endl;
  if (control_mode == MODE_ABC && !gg_is_walking && adjust_footstep_interpolator->isEmpty()) {
      Guard guard(m_mutex);
      //
      hrp::Vector3 eepos, org_mid_rpy, target_mid_rpy;
      hrp::Matrix33 eerot, tmprot;
      coordinates org_mid_coords, target_mid_coords;
      // Get org coords
      ikp["rleg"].adjust_interpolation_org_p0 = ikp["rleg"].target_p0;
      ikp["lleg"].adjust_interpolation_org_p0 = ikp["lleg"].target_p0;
      ikp["rleg"].adjust_interpolation_org_r0 = ikp["rleg"].target_r0;
      ikp["lleg"].adjust_interpolation_org_r0 = ikp["lleg"].target_r0;
      mid_coords(org_mid_coords, 0.5,
                 coordinates(ikp["rleg"].adjust_interpolation_org_p0, ikp["rleg"].adjust_interpolation_org_r0),
                 coordinates(ikp["lleg"].adjust_interpolation_org_p0, ikp["lleg"].adjust_interpolation_org_r0));
      org_mid_rpy = hrp::rpyFromRot(org_mid_coords.rot);
      // Get target coords
      //   Input : ee coords
      //   Output : link coords
      memcpy(eepos.data(), rfootstep.pos, sizeof(double)*3);
      eerot = (Eigen::Quaternion<double>(rfootstep.rot[0], rfootstep.rot[1], rfootstep.rot[2], rfootstep.rot[3])).normalized().toRotationMatrix(); // rtc: 
      ikp["rleg"].adjust_interpolation_target_r0 = eerot * ikp["rleg"].localR.transpose();
      ikp["rleg"].adjust_interpolation_target_p0 = eepos - ikp["rleg"].adjust_interpolation_target_r0 * ikp["rleg"].localPos;
      memcpy(eepos.data(), lfootstep.pos, sizeof(double)*3);
      eerot = (Eigen::Quaternion<double>(lfootstep.rot[0], lfootstep.rot[1], lfootstep.rot[2], lfootstep.rot[3])).normalized().toRotationMatrix(); // rtc: 
      ikp["lleg"].adjust_interpolation_target_r0 = eerot * ikp["lleg"].localR.transpose();
      ikp["lleg"].adjust_interpolation_target_p0 = eepos - ikp["lleg"].adjust_interpolation_target_r0 * ikp["lleg"].localPos;
      mid_coords(target_mid_coords, 0.5,
                 coordinates(ikp["rleg"].adjust_interpolation_target_p0, ikp["rleg"].adjust_interpolation_target_r0),
                 coordinates(ikp["lleg"].adjust_interpolation_target_p0, ikp["lleg"].adjust_interpolation_target_r0));
      coordinates rtrans, ltrans;
      target_mid_coords.transformation(rtrans, coordinates(ikp["rleg"].adjust_interpolation_target_p0, ikp["rleg"].adjust_interpolation_target_r0));
      target_mid_coords.transformation(ltrans, coordinates(ikp["lleg"].adjust_interpolation_target_p0, ikp["lleg"].adjust_interpolation_target_r0));
      target_mid_rpy = hrp::rpyFromRot(target_mid_coords.rot);
      // Fix target pos => org pos, target yaw => org yaw
      target_mid_rpy(2) = org_mid_rpy(2);
      target_mid_coords.rot = hrp::rotFromRpy(target_mid_rpy);
      target_mid_coords.pos = org_mid_coords.pos;
      // Calculate rleg and lleg coords
      coordinates tmpc;
      tmpc = target_mid_coords;
      tmpc.transform(rtrans);
      ikp["rleg"].adjust_interpolation_target_p0 = tmpc.pos;
      ikp["rleg"].adjust_interpolation_target_r0 = tmpc.rot;
      tmpc = target_mid_coords;
      tmpc.transform(ltrans);
      ikp["lleg"].adjust_interpolation_target_p0 = tmpc.pos;
      ikp["lleg"].adjust_interpolation_target_r0 = tmpc.rot;
      // Set interpolator
      adjust_footstep_interpolator->clear();
      double tmp = 0.0;
      adjust_footstep_interpolator->set(&tmp);
      tmp = 1.0;
      adjust_footstep_interpolator->go(&tmp, adjust_footstep_transition_time, true);
  }
  while (!adjust_footstep_interpolator->isEmpty() )
    usleep(1000);
  usleep(1000);
  return true;
};

bool AutoBalancer::getRemainingFootstepSequence(OpenHRP::AutoBalancerService::FootstepSequence_out o_footstep, CORBA::Long& o_current_fs_idx)
{
    std::cerr << "[" << m_profile.instance_name << "] getRemainingFootstepSequence" << std::endl;
    o_footstep = new OpenHRP::AutoBalancerService::FootstepSequence;
    if (gg_is_walking) {
        std::vector< std::vector<step_node> > fsnl = gg->get_remaining_footstep_nodes_list();
        o_current_fs_idx = gg->get_footstep_index();
        o_footstep->length(fsnl.size());
        for (size_t i = 0; i < fsnl.size(); i++) {
            o_footstep[i].leg = (fsnl[i].front().l_r==RLEG?"rleg":"lleg");
            copyRatscoords2Footstep(o_footstep[i], fsnl[i].front().worldcoords);
        }
    }
};

void AutoBalancer::static_balance_point_proc_one(hrp::Vector3& tmp_input_sbp, const double ref_com_height)
{
  hrp::Vector3 target_sbp = hrp::Vector3(0, 0, 0);
  hrp::Vector3 tmpcog = m_robot->calcCM();
  switch ( use_force ) {
  case MODE_REF_FORCE:
    calc_static_balance_point_from_forces(target_sbp, tmpcog, ref_com_height, ref_forces);
    tmp_input_sbp = target_sbp - sbp_offset;
    sbp_cog_offset = tmp_input_sbp - tmpcog;
    break;
  case MODE_NO_FORCE:
    tmp_input_sbp = tmpcog + sbp_cog_offset;
    break;
  default: break;
  }
};

void AutoBalancer::calc_static_balance_point_from_forces(hrp::Vector3& sb_point, const hrp::Vector3& tmpcog, const double ref_com_height, std::vector<hrp::Vector3>& tmp_forces)
{
  hrp::Vector3 denom, nume;
  /* sb_point[m] = nume[kg * m/s^2 * m] / denom[kg * m/s^2] */
  double mass = m_robot->totalMass();
  for (size_t j = 0; j < 2; j++) {
    nume(j) = mass * gg->get_gravitational_acceleration() * tmpcog(j);
    denom(j) = mass * gg->get_gravitational_acceleration();
    for (size_t i = 0; i < sensor_names.size(); i++) {
      if ( sensor_names[i].find("hsensor") != std::string::npos || sensor_names[i].find("asensor") != std::string::npos ) { // tempolary to get arm force coords
          hrp::Link* parentlink;
          hrp::ForceSensor* sensor = m_robot->sensor<hrp::ForceSensor>(sensor_names[i]);
          if (sensor) parentlink = sensor->link;
          else parentlink = m_vfs[sensor_names[i]].link;
          for ( std::map<std::string, ABCIKparam>::iterator it = ikp.begin(); it != ikp.end(); it++ ) {
              if (it->second.target_link->name == parentlink->name) {
                  hrp::Vector3 fpos = parentlink->p + parentlink->R * it->second.localPos;
                  nume(j) += ( (fpos(2) - ref_com_height) * tmp_forces[i](j) - fpos(j) * tmp_forces[i](2) );
                  denom(j) -= tmp_forces[i](2);
              }
          }
      }
    }
    sb_point(j) = nume(j) / denom(j);
  }
  sb_point(2) = ref_com_height;
};

#ifndef rad2deg
#define rad2deg(rad) (rad * 180 / M_PI)
#endif
#ifndef deg2rad
#define deg2rad(deg) (deg * M_PI / 180)
#endif

hrp::Vector3 AutoBalancer::calc_vel_from_hand_error (const coordinates& tmp_fix_coords)
{
  if (graspless_manip_mode) {
    hrp::Vector3 dp,dr;
    coordinates ref_hand_coords(gg->get_dst_foot_midcoords()), act_hand_coords;
    ref_hand_coords.transform(graspless_manip_reference_trans_coords); // desired arm coords
    hrp::Vector3 foot_pos(gg->get_dst_foot_midcoords().pos);
    if ( graspless_manip_arm == "arms" ) {
      // act_hand_coords.pos = (target_coords["rarm"].pos + target_coords["larm"].pos) / 2.0;
      // vector3 cur_y(target_coords["larm"].pos - target_coords["rarm"].pos);
      // cur_y(2) = 0;
      // alias(cur_y) = normalize(cur_y);
      // vector3 ref_y(ref_hand_coords.axis(AXIS_Y));
      // ref_y(2) = 0;
      // alias(ref_y) = normalize(ref_y);
      // dr = 0,0,((vector3(cross(ref_y, cur_y))(2) > 0 ? 1.0 : -1.0) * std::acos(dot(ref_y, cur_y))); // fix for rotation
    } else {
      ABCIKparam& tmpikp = ikp[graspless_manip_arm];
      act_hand_coords = rats::coordinates(tmpikp.target_p0 + tmpikp.target_r0 * tmpikp.localPos,
                                          tmpikp.target_r0 * tmpikp.localR);
      rats::difference_rotation(dr, ref_hand_coords.rot, act_hand_coords.rot);
      dr(0) = 0; dr(1) = 0;
    }
    dp = act_hand_coords.pos - ref_hand_coords.pos
        + dr.cross(hrp::Vector3(foot_pos - act_hand_coords.pos));
    dp(2) = 0;
    hrp::Matrix33 foot_mt(gg->get_dst_foot_midcoords().rot.transpose());
    //alias(dp) = foot_mt * dp;
    hrp::Vector3 dp2 = foot_mt * dp;
    //alias(dr) = foot_mt * dr;
    return hrp::Vector3(graspless_manip_p_gain[0] * dp2(0)/gg->get_default_step_time(),
                        graspless_manip_p_gain[1] * dp2(1)/gg->get_default_step_time(),
                        graspless_manip_p_gain[2] * rad2deg(dr(2))/gg->get_default_step_time());
  } else {
    return hrp::Vector3::Zero();
  }
};

//
extern "C"
{

    void AutoBalancerInit(RTC::Manager* manager)
    {
        RTC::Properties profile(autobalancer_spec);
        manager->registerFactory(profile,
                                 RTC::Create<AutoBalancer>,
                                 RTC::Delete<AutoBalancer>);
    }

};


