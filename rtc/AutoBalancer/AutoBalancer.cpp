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
#include "hrpsys/util/Hrpsys.h"


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

AutoBalancer::AutoBalancer(RTC::Manager* manager)
    : RTC::DataFlowComponentBase(manager),
      // <rtc-template block="initializer">
      m_qRefIn("qRef", m_qRef),
      m_basePosIn("basePosIn", m_basePos),
      m_baseRpyIn("baseRpyIn", m_baseRpy),
      m_zmpIn("zmpIn", m_zmp),
      m_optionalDataIn("optionalData", m_optionalData),
      m_emergencySignalIn("emergencySignal", m_emergencySignal),
      m_diffCPIn("diffCapturePoint", m_diffCP),
      m_refFootOriginExtMomentIn("refFootOriginExtMoment", m_refFootOriginExtMoment),
      m_refFootOriginExtMomentIsHoldValueIn("refFootOriginExtMomentIsHoldValue", m_refFootOriginExtMomentIsHoldValue),
      m_actContactStatesIn("actContactStates", m_actContactStates),
      m_qOut("q", m_qRef),
      m_zmpOut("zmpOut", m_zmp),
      m_basePosOut("basePosOut", m_basePos),
      m_baseRpyOut("baseRpyOut", m_baseRpy),
      m_baseTformOut("baseTformOut", m_baseTform),
      m_basePoseOut("basePoseOut", m_basePose),
      m_accRefOut("accRef", m_accRef),
      m_contactStatesOut("contactStates", m_contactStates),
      m_toeheelRatioOut("toeheelRatio", m_toeheelRatio),
      m_controlSwingSupportTimeOut("controlSwingSupportTime", m_controlSwingSupportTime),
      m_walkingStatesOut("walkingStates", m_walkingStates),
      m_sbpCogOffsetOut("sbpCogOffset", m_sbpCogOffset),
      m_cogOut("cogOut", m_cog),
      m_AutoBalancerServicePort("AutoBalancerService"),
      // </rtc-template>
      gait_type(BIPED),
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
    addInPort("diffCapturePoint", m_diffCPIn);
    addInPort("actContactStates", m_actContactStatesIn);
    addInPort("refFootOriginExtMoment", m_refFootOriginExtMomentIn);
    addInPort("refFootOriginExtMomentIsHoldValue", m_refFootOriginExtMomentIsHoldValueIn);

    // Set OutPort buffer
    addOutPort("q", m_qOut);
    addOutPort("zmpOut", m_zmpOut);
    addOutPort("basePosOut", m_basePosOut);
    addOutPort("baseRpyOut", m_baseRpyOut);
    addOutPort("baseTformOut", m_baseTformOut);
    addOutPort("basePoseOut", m_basePoseOut);
    addOutPort("accRef", m_accRefOut);
    addOutPort("contactStates", m_contactStatesOut);
    addOutPort("toeheelRatio", m_toeheelRatioOut);
    addOutPort("controlSwingSupportTime", m_controlSwingSupportTimeOut);
    addOutPort("cogOut", m_cogOut);
    addOutPort("walkingStates", m_walkingStatesOut);
    addOutPort("sbpCogOffset", m_sbpCogOffsetOut);
  
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
      std::cerr << "[" << m_profile.instance_name << "] failed to load model[" << prop["model"] << "]" << std::endl;
      return RTC::RTC_ERROR;
    }

    // allocate memory for outPorts
    m_qRef.data.length(m_robot->numJoints());
    m_baseTform.data.length(12);

    control_mode = MODE_IDLE;
    loop = 0;

    // setting from conf file
    // GaitGenerator requires abc_leg_offset and abc_stride_parameter in robot conf file
    // setting leg_pos from conf file
    coil::vstring leg_offset_str = coil::split(prop["abc_leg_offset"], ",");
    leg_names.push_back("rleg");
    leg_names.push_back("lleg");

    // Generate FIK
    fik = fikPtr(new SimpleFullbodyInverseKinematicsSolver(m_robot, std::string(m_profile.instance_name), m_dt));

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
        hrp::Link* root = m_robot->link(ee_target);
        for (size_t j = 0; j < 3; j++) {
          coil::stringTo(tp.localPos(j), end_effectors_str[i*prop_num+3+j].c_str());
        }
        double tmpv[4];
        for (int j = 0; j < 4; j++ ) {
          coil::stringTo(tmpv[j], end_effectors_str[i*prop_num+6+j].c_str());
        }
        tp.localR = Eigen::AngleAxis<double>(tmpv[3], hrp::Vector3(tmpv[0], tmpv[1], tmpv[2])).toRotationMatrix(); // rotation in VRML is represented by axis + angle
        // FIK param
        SimpleFullbodyInverseKinematicsSolver::IKparam tmp_fikp;
        tmp_fikp.manip = hrp::JointPathExPtr(new hrp::JointPathEx(m_robot, m_robot->link(ee_base), m_robot->link(ee_target), m_dt, false, std::string(m_profile.instance_name)));
        tmp_fikp.target_link = m_robot->link(ee_target);
        tmp_fikp.localPos = tp.localPos;
        tmp_fikp.localR = tp.localR;
        tmp_fikp.max_limb_length = 0.0;
        while (!root->isRoot()) {
          tmp_fikp.max_limb_length += root->b.norm();
          tmp_fikp.parent_name = root->name;
          root = root->parent;
        }
        fik->ikp.insert(std::pair<std::string, SimpleFullbodyInverseKinematicsSolver::IKparam>(ee_name, tmp_fikp));
        // Fix for toe joint
        //   Toe joint is defined as end-link joint in the case that end-effector link != force-sensor link
        //   Without toe joints, "end-effector link == force-sensor link" is assumed.
        //   With toe joints, "end-effector link != force-sensor link" is assumed.
        if (m_robot->link(ee_target)->sensors.size() == 0) { // If end-effector link has no force sensor
            std::vector<double> optw(fik->ikp[ee_name].manip->numJoints(), 1.0);
            optw.back() = 0.0; // Set weight = 0 for toe joint by default
            fik->ikp[ee_name].manip->setOptionalWeightVector(optw);
            tp.has_toe_joint = true;
        } else {
            tp.has_toe_joint = false;
        }
        tp.target_link = m_robot->link(ee_target);
        ikp.insert(std::pair<std::string, ABCIKparam>(ee_name , tp));
        ee_vec.push_back(ee_name);
        std::cerr << "[" << m_profile.instance_name << "] End Effector [" << ee_name << "]" << std::endl;
        std::cerr << "[" << m_profile.instance_name << "]   target = " << ikp[ee_name].target_link->name << ", base = " << ee_base << std::endl;
        std::cerr << "[" << m_profile.instance_name << "]   offset_pos = " << tp.localPos.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "    [", "]")) << "[m]" << std::endl;
        std::cerr << "[" << m_profile.instance_name << "]   has_toe_joint = " << (tp.has_toe_joint?"true":"false") << std::endl;
        contact_states_index_map.insert(std::pair<std::string, size_t>(ee_name, i));
      }
      m_contactStates.data.length(num);
      m_toeheelRatio.data.length(num);
      if (ikp.find("rleg") != ikp.end() && ikp.find("lleg") != ikp.end()) {
        m_contactStates.data[contact_states_index_map["rleg"]] = true;
        m_contactStates.data[contact_states_index_map["lleg"]] = true;
      }
      if (ikp.find("rarm") != ikp.end() && ikp.find("larm") != ikp.end()) {
        m_contactStates.data[contact_states_index_map["rarm"]] = false;
        m_contactStates.data[contact_states_index_map["larm"]] = false;
      }
      m_controlSwingSupportTime.data.length(num);
      for (size_t i = 0; i < num; i++) m_controlSwingSupportTime.data[i] = 1.0;
      for (size_t i = 0; i < num; i++) m_toeheelRatio.data[i] = rats::no_using_toe_heel_ratio;
    }
    std::vector<hrp::Vector3> leg_pos;
    if (leg_offset_str.size() > 0) {
      hrp::Vector3 leg_offset;
      for (size_t i = 0; i < 3; i++) coil::stringTo(leg_offset(i), leg_offset_str[i].c_str());
      std::cerr << "[" << m_profile.instance_name << "] abc_leg_offset = " << leg_offset.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "    [", "]")) << "[m]" << std::endl;
      leg_pos.push_back(hrp::Vector3(-1*leg_offset));
      leg_pos.push_back(hrp::Vector3(leg_offset));
    }
    if (leg_pos.size() < ikp.size()) {
        size_t tmp_leg_pos_size = leg_pos.size();
        for (size_t i = 0; i < ikp.size() - tmp_leg_pos_size; i++) {
            leg_pos.push_back(hrp::Vector3::Zero());
        }
    }

    std::vector<std::pair<hrp::Link*, hrp::Link*> > interlocking_joints;
    readInterlockingJointsParamFromProperties(interlocking_joints, m_robot, prop["interlocking_joints"], std::string(m_profile.instance_name));
    if (interlocking_joints.size() > 0) {
        fik->initializeInterlockingJoints(interlocking_joints);
    }

    zmp_offset_interpolator = new interpolator(ikp.size()*3, m_dt);
    zmp_offset_interpolator->setName(std::string(m_profile.instance_name)+" zmp_offset_interpolator");
    zmp_transition_time = 1.0;
    transition_interpolator = new interpolator(1, m_dt, interpolator::HOFFARBIB, 1);
    transition_interpolator->setName(std::string(m_profile.instance_name)+" transition_interpolator");
    transition_interpolator_ratio = 0.0;
    adjust_footstep_interpolator = new interpolator(1, m_dt, interpolator::HOFFARBIB, 1);
    adjust_footstep_interpolator->setName(std::string(m_profile.instance_name)+" adjust_footstep_interpolator");
    transition_time = 2.0;
    adjust_footstep_transition_time = 2.0;
    leg_names_interpolator = new interpolator(1, m_dt, interpolator::HOFFARBIB, 1);
    leg_names_interpolator->setName(std::string(m_profile.instance_name)+" leg_names_interpolator");
    leg_names_interpolator_ratio = 1.0;

    // setting stride limitations from conf file
    double stride_fwd_x_limit = 0.15;
    double stride_outside_y_limit = 0.05;
    double stride_outside_th_limit = 10;
    double stride_bwd_x_limit = 0.05;
    double stride_inside_y_limit = stride_outside_y_limit*0.5;
    double stride_inside_th_limit = stride_outside_th_limit*0.5;
    std::cerr << "[" << m_profile.instance_name << "] abc_stride_parameter : " << stride_fwd_x_limit << "[m], " << stride_outside_y_limit << "[m], " << stride_outside_th_limit << "[deg], " << stride_bwd_x_limit << "[m]" << std::endl;
    if (default_zmp_offsets.size() == 0) {
      for (size_t i = 0; i < ikp.size(); i++) default_zmp_offsets.push_back(hrp::Vector3::Zero());
    }
    if (leg_offset_str.size() > 0) {
      gg = ggPtr(new rats::gait_generator(m_dt, leg_pos, leg_names, stride_fwd_x_limit/*[m]*/, stride_outside_y_limit/*[m]*/, stride_outside_th_limit/*[deg]*/,
                                          stride_bwd_x_limit/*[m]*/, stride_inside_y_limit/*[m]*/, stride_inside_th_limit/*[m]*/));
      gg->set_default_zmp_offsets(default_zmp_offsets);
    }
    gg_is_walking = gg_solved = false;
    m_walkingStates.data = false;
    fix_leg_coords = coordinates();

    // load virtual force sensors
    readVirtualForceSensorParamFromProperties(m_vfs, m_robot, prop["virtual_force_sensor"], std::string(m_profile.instance_name));
    // ref force port
    unsigned int npforce = m_robot->numSensors(hrp::Sensor::FORCE);
    unsigned int nvforce = m_vfs.size();
    unsigned int nforce  = npforce + nvforce;
    // check number of force sensors
    if (nforce < m_contactStates.data.length()) {
        std::cerr << "[" << m_profile.instance_name << "] WARNING! This robot model has less force sensors(" << nforce;
        std::cerr << ") than end-effector settings(" << m_contactStates.data.length() << ") !" << std::endl;
    }

    m_ref_force.resize(nforce);
    m_ref_forceIn.resize(nforce);
    m_force.resize(nforce);
    m_ref_forceOut.resize(nforce);
    m_limbCOPOffset.resize(nforce);
    m_limbCOPOffsetOut.resize(nforce);
    for (unsigned int i=0; i<npforce; i++){
        sensor_names.push_back(m_robot->sensor(hrp::Sensor::FORCE, i)->name);
    }
    for (unsigned int i=0; i<nvforce; i++){
        for ( std::map<std::string, hrp::VirtualForceSensorParam>::iterator it = m_vfs.begin(); it != m_vfs.end(); it++ ) {
            if (it->second.id == (int)i) sensor_names.push_back(it->first);
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
    // set force port
    for (unsigned int i=0; i<nforce; i++){
        m_ref_forceOut[i] = new OutPort<TimedDoubleSeq>(std::string(sensor_names[i]).c_str(), m_force[i]);
        m_force[i].data.length(6);
        m_force[i].data[0] = m_force[i].data[1] = m_force[i].data[2] = 0.0;
        m_force[i].data[3] = m_force[i].data[4] = m_force[i].data[5] = 0.0;
        registerOutPort(std::string(sensor_names[i]).c_str(), *m_ref_forceOut[i]);
        std::cerr << "[" << m_profile.instance_name << "]   name = " << std::string(sensor_names[i]) << std::endl;
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
    is_hand_fix_mode = false;

    hrp::Sensor* sen = m_robot->sensor<hrp::RateGyroSensor>("gyrometer");
    if (sen == NULL) {
        std::cerr << "[" << m_profile.instance_name << "] WARNING! This robot model has no GyroSensor named 'gyrometer'! " << std::endl;
    }

    additional_force_applied_link = m_robot->rootLink();
    additional_force_applied_point_offset = hrp::Vector3::Zero();
    return RTC::RTC_OK;
}



RTC::ReturnCode_t AutoBalancer::onFinalize()
{
  delete zmp_offset_interpolator;
  delete transition_interpolator;
  delete adjust_footstep_interpolator;
  delete leg_names_interpolator;
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
    transition_interpolator->setGoal(&tmp_ratio, m_dt, true); // sync in one controller loop
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

    // Read Inport
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
    }
    if (m_emergencySignalIn.isNew()){
        m_emergencySignalIn.read();
        // if (!is_stop_mode) {
        //     std::cerr << "[" << m_profile.instance_name << "] emergencySignal is set!" << std::endl;
        //     is_stop_mode = true;
        //     gg->emergency_stop();
        // }
    }
    if (m_diffCPIn.isNew()) {
      m_diffCPIn.read();
      gg->set_diff_cp(hrp::Vector3(m_diffCP.data.x, m_diffCP.data.y, m_diffCP.data.z));
    }
    if (m_refFootOriginExtMomentIn.isNew()) {
      m_refFootOriginExtMomentIn.read();
    }
    if (m_refFootOriginExtMomentIsHoldValueIn.isNew()) {
      m_refFootOriginExtMomentIsHoldValueIn.read();
    }
    if (m_actContactStatesIn.isNew()) {
      m_actContactStatesIn.read();
      std::vector<bool> tmp_contacts(m_actContactStates.data.length());
      for (size_t i = 0; i < m_actContactStates.data.length(); i++) {
        tmp_contacts[i] = m_actContactStates.data[i];
      }
      gg->set_act_contact_states(tmp_contacts);
    }

    // Calculation
    Guard guard(m_mutex);
    hrp::Vector3 ref_basePos;
    hrp::Matrix33 ref_baseRot;
    hrp::Vector3 rel_ref_zmp; // ref zmp in base frame
    if ( is_legged_robot ) {
      // For parameters
      fik->storeCurrentParameters();
      getTargetParameters();
      // Get transition ratio
      bool is_transition_interpolator_empty = transition_interpolator->isEmpty();
      if (!is_transition_interpolator_empty) {
        transition_interpolator->get(&transition_interpolator_ratio, true);
      } else {
        transition_interpolator_ratio = (control_mode == MODE_IDLE) ? 0.0 : 1.0;
      }
      if (control_mode != MODE_IDLE ) {
        solveFullbodyIK();
//        /////// Inverse Dynamics /////////
//        if(!idsb.is_initialized){
//          idsb.setInitState(m_robot, m_dt);
//          invdyn_zmp_filters.resize(3);
//          for(int i=0;i<3;i++){
//            invdyn_zmp_filters[i].setParameterAsBiquad(25, 1/std::sqrt(2), 1.0/m_dt);
//            invdyn_zmp_filters[i].reset(ref_zmp(i));
//          }
//        }
//        calcAccelerationsForInverseDynamics(m_robot, idsb);
//        if(gg_is_walking){
//          calcWorldZMPFromInverseDynamics(m_robot, idsb, ref_zmp);
//          for(int i=0;i<3;i++) ref_zmp(i) = invdyn_zmp_filters[i].passFilter(ref_zmp(i));
//        }
//        updateInvDynStateBuffer(idsb);

        rel_ref_zmp = m_robot->rootLink()->R.transpose() * (ref_zmp - m_robot->rootLink()->p);
      } else {
        rel_ref_zmp = input_zmp;
        fik->d_root_height = 0.0;
      }
      // Transition
      if (!is_transition_interpolator_empty) {
        // transition_interpolator_ratio 0=>1 : IDLE => ABC
        // transition_interpolator_ratio 1=>0 : ABC => IDLE
        ref_basePos = (1-transition_interpolator_ratio) * input_basePos + transition_interpolator_ratio * m_robot->rootLink()->p;
        rel_ref_zmp = (1-transition_interpolator_ratio) * input_zmp + transition_interpolator_ratio * rel_ref_zmp;
        rats::mid_rot(ref_baseRot, transition_interpolator_ratio, input_baseRot, m_robot->rootLink()->R);
        for ( unsigned int i = 0; i < m_robot->numJoints(); i++ ) {
          m_robot->joint(i)->q = (1-transition_interpolator_ratio) * m_qRef.data[i] + transition_interpolator_ratio * m_robot->joint(i)->q;
        }
        for (unsigned int i=0; i< m_force.size(); i++) {
            for (unsigned int j=0; j<6; j++) {
                m_force[i].data[j] = transition_interpolator_ratio * m_force[i].data[j] + (1-transition_interpolator_ratio) * m_ref_force[i].data[j];
            }
        }
        for (unsigned int i=0; i< m_limbCOPOffset.size(); i++) {
            // transition (TODO:set stopABCmode value instead of 0)
            m_limbCOPOffset[i].data.x = transition_interpolator_ratio * m_limbCOPOffset[i].data.x;// + (1-transition_interpolator_ratio) * 0;
            m_limbCOPOffset[i].data.y = transition_interpolator_ratio * m_limbCOPOffset[i].data.y;// + (1-transition_interpolator_ratio) * 0;
            m_limbCOPOffset[i].data.z = transition_interpolator_ratio * m_limbCOPOffset[i].data.z;// + (1-transition_interpolator_ratio) * 0;
        }
      } else {
        ref_basePos = m_robot->rootLink()->p;
        ref_baseRot = m_robot->rootLink()->R;
      }
      // mode change for sync
      if (control_mode == MODE_SYNC_TO_ABC) {
        control_mode = MODE_ABC;
      } else if (control_mode == MODE_SYNC_TO_IDLE && transition_interpolator->isEmpty() ) {
        std::cerr << "[" << m_profile.instance_name << "] [" << m_qRef.tm
                  << "] Finished cleanup" << std::endl;
        control_mode = MODE_IDLE;
      }
    }

    // Write Outport
    if ( m_qRef.data.length() != 0 ) { // initialized
      if (is_legged_robot) {
        for ( unsigned int i = 0; i < m_robot->numJoints(); i++ ){
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
      // sbpCogOffset
      m_sbpCogOffset.data.x = sbp_cog_offset(0);
      m_sbpCogOffset.data.y = sbp_cog_offset(1);
      m_sbpCogOffset.data.z = sbp_cog_offset(2);
      m_sbpCogOffset.tm = m_qRef.tm;
      // write
      m_basePosOut.write();
      m_baseRpyOut.write();
      m_baseTformOut.write();
      m_basePoseOut.write();
      m_zmpOut.write();
      m_cogOut.write();
      m_sbpCogOffsetOut.write();

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
      m_toeheelRatio.tm = m_qRef.tm;
      m_toeheelRatioOut.write();
      m_walkingStates.data = gg_is_walking;
      m_walkingStates.tm = m_qRef.tm;
      m_walkingStatesOut.write();

      for (unsigned int i=0; i<m_ref_forceOut.size(); i++){
          m_force[i].tm = m_qRef.tm;
          m_ref_forceOut[i]->write();
      }

      for (unsigned int i=0; i<m_limbCOPOffsetOut.size(); i++){
          m_limbCOPOffset[i].tm = m_qRef.tm;
          m_limbCOPOffsetOut[i]->write();
      }
    }

    return RTC::RTC_OK;
}

void AutoBalancer::getTargetParameters()
{
  // joint angles
  for ( unsigned int i = 0; i < m_robot->numJoints(); i++ ){
    m_robot->joint(i)->q = m_qRef.data[i];
  }
  fik->setReferenceJointAngles();
  // basepos, rot, zmp
  m_robot->rootLink()->p = input_basePos;
  m_robot->rootLink()->R = input_baseRot;
  m_robot->calcForwardKinematics();
  gg->proc_zmp_weight_map_interpolation();
  if (control_mode != MODE_IDLE) {
    interpolateLegNamesAndZMPOffsets();
    // Calculate tmp_fix_coords and something
    coordinates tmp_fix_coords;
    if ( gg_is_walking ) {
      gg->set_default_zmp_offsets(default_zmp_offsets);
      gg_solved = gg->proc_one_tick();
      gg->get_swing_support_mid_coords(tmp_fix_coords);
    } else {
      tmp_fix_coords = fix_leg_coords;
    }
    if (!adjust_footstep_interpolator->isEmpty()) {
        calcFixCoordsForAdjustFootstep(tmp_fix_coords);
        fix_leg_coords = tmp_fix_coords;
    }
    // TODO : see explanation in this function
    fixLegToCoords2(tmp_fix_coords);
    fix_leg_coords2 = tmp_fix_coords;

    // Get output parameters and target EE coords
    target_root_p = m_robot->rootLink()->p;
    target_root_R = m_robot->rootLink()->R;
    if ( gg_is_walking ) {
      getOutputParametersForWalking();
    } else {
      getOutputParametersForABC();
    }
    //   Just for ik initial value
    if (control_mode == MODE_SYNC_TO_ABC) {
        fik->current_root_p = target_root_p;
        fik->current_root_R = target_root_R;
        for ( std::map<std::string, ABCIKparam>::iterator it = ikp.begin(); it != ikp.end(); it++ ) {
            if ( std::find(leg_names.begin(), leg_names.end(), it->first) != leg_names.end() ) {
                it->second.target_p0 = it->second.target_link->p + it->second.target_link->R * it->second.localPos;
                it->second.target_r0 = it->second.target_link->R * it->second.localR;
            }
        }
    }

    // Calculate other parameters
    updateTargetCoordsForHandFixMode (tmp_fix_coords);
    rotateRefForcesForFixCoords (tmp_fix_coords);
    // TODO : see explanation in this function
    calculateOutputRefForces ();
    // TODO : see explanation in this function
    updateWalkingVelocityFromHandError(tmp_fix_coords);
    calcReferenceJointAnglesForIK();

    // Calculate ZMP, COG, and sbp targets
    hrp::Vector3 tmp_ref_cog(m_robot->calcCM());
    hrp::Vector3 tmp_foot_mid_pos = calcFootMidPosUsingZMPWeightMap ();
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
  } else { // MODE_IDLE
      // Update fix_leg_coords based on input basePos and rot if MODE_IDLE
      std::vector<coordinates> tmp_end_coords_list;
      for ( std::map<std::string, ABCIKparam>::iterator it = ikp.begin(); it != ikp.end(); it++ ) {
          if ( std::find(leg_names.begin(), leg_names.end(), it->first) != leg_names.end() ) {
              tmp_end_coords_list.push_back(coordinates(it->second.target_link->p+it->second.target_link->R*it->second.localPos, it->second.target_link->R*it->second.localR));
          }
      }
      multi_mid_coords(fix_leg_coords, tmp_end_coords_list);
      getOutputParametersForIDLE();
      // Set force
      for (unsigned int i=0; i< m_force.size(); i++) {
          for (unsigned int j=0; j<6; j++) {
              m_force[i].data[j] = m_ref_force[i].data[j];
          }
      }
  }
  // Just for stop walking
  if (gg_is_walking && !gg_solved) stopWalking ();
};

void AutoBalancer::getOutputParametersForWalking ()
{
    for (std::map<std::string, ABCIKparam>::iterator it = ikp.begin(); it != ikp.end(); it++) {
        size_t idx = contact_states_index_map[it->first];
        // Check whether "it->first" ee_name is included in leg_names. leg_names is equivalent to "swing" + "support" in gg.
        if (std::find(leg_names.begin(), leg_names.end(), it->first) != leg_names.end()) {
            // Set EE coords
            gg->get_swing_support_ee_coords_from_ee_name(it->second.target_p0, it->second.target_r0, it->first);
            // Set contactStates
            m_contactStates.data[idx] = gg->get_current_support_state_from_ee_name(it->first);
            // Set controlSwingSupportTime
            m_controlSwingSupportTime.data[idx] = gg->get_current_swing_time_from_ee_name(it->first);
            // Set limbCOPOffset
            hrp::Vector3 tmpzmpoff(m_limbCOPOffset[idx].data.x, m_limbCOPOffset[idx].data.y, m_limbCOPOffset[idx].data.z);
            gg->get_swing_support_foot_zmp_offsets_from_ee_name(tmpzmpoff, it->first);
            m_limbCOPOffset[idx].data.x = tmpzmpoff(0);
            m_limbCOPOffset[idx].data.y = tmpzmpoff(1);
            m_limbCOPOffset[idx].data.z = tmpzmpoff(2);
            // Set toe heel ratio which can be used force moment distribution
            double tmp = m_toeheelRatio.data[idx];
            gg->get_current_toe_heel_ratio_from_ee_name(tmp, it->first);
            m_toeheelRatio.data[idx] = tmp;
        } else { // Not included in leg_names
            // Set EE coords
            it->second.target_p0 = it->second.target_link->p + it->second.target_link->R * it->second.localPos;
            it->second.target_r0 = it->second.target_link->R * it->second.localR;
            // contactStates is OFF other than leg_names
            m_contactStates.data[idx] = false;
            // controlSwingSupportTime is not used while double support period, 1.0 is neglected
            m_controlSwingSupportTime.data[idx] = 1.0;
            // Set limbCOPOffset
            m_limbCOPOffset[idx].data.x = default_zmp_offsets[idx](0);
            m_limbCOPOffset[idx].data.y = default_zmp_offsets[idx](1);
            m_limbCOPOffset[idx].data.z = default_zmp_offsets[idx](2);
            // Set toe heel ratio which can be used force moment distribution
            m_toeheelRatio.data[idx] = rats::no_using_toe_heel_ratio;
        }
    }
};

void AutoBalancer::getOutputParametersForABC ()
{
    // double support by default
    for (std::map<std::string, ABCIKparam>::iterator it = ikp.begin(); it != ikp.end(); it++) {
        size_t idx = contact_states_index_map[it->first];
        // Set EE coords.
        //   If not included in leg_names, calculate EE coords because of not being used in GaitGenerator.
        //   Otherwise, keep previous EE coords derived from GaitGenerator or initial value.
        if ( std::find(leg_names.begin(), leg_names.end(), it->first) == leg_names.end() ) {
            it->second.target_p0 = it->second.target_link->p + it->second.target_link->R * it->second.localPos;
            it->second.target_r0 = it->second.target_link->R * it->second.localR;
        }
        // Set contactStates
        std::vector<std::string>::const_iterator dst = std::find_if(leg_names.begin(), leg_names.end(), (boost::lambda::_1 == it->first));
        if (dst != leg_names.end()) {
            m_contactStates.data[idx] = true;
        } else {
            m_contactStates.data[idx] = false;
        }
        // controlSwingSupportTime is not used while double support period, 1.0 is neglected
        m_controlSwingSupportTime.data[idx] = 1.0;
        // Set limbCOPOffset
        m_limbCOPOffset[idx].data.x = default_zmp_offsets[idx](0);
        m_limbCOPOffset[idx].data.y = default_zmp_offsets[idx](1);
        m_limbCOPOffset[idx].data.z = default_zmp_offsets[idx](2);
        // Set toe heel ratio is not used while double support
        m_toeheelRatio.data[idx] = rats::no_using_toe_heel_ratio;
    }
};

void AutoBalancer::getOutputParametersForIDLE ()
{
    // Set contactStates and controlSwingSupportTime
    if (m_optionalData.data.length() >= contact_states_index_map.size()*2) {
        // current optionalData is contactstates x limb and controlSwingSupportTime x limb
        //   If contactStates in optionalData is 1.0, m_contactStates is true. Otherwise, false.
        for ( std::map<std::string, ABCIKparam>::iterator it = ikp.begin(); it != ikp.end(); it++ ) {
            m_contactStates.data[contact_states_index_map[it->first]] = isOptionalDataContact(it->first);
            m_controlSwingSupportTime.data[contact_states_index_map[it->first]] = m_optionalData.data[contact_states_index_map[it->first]+contact_states_index_map.size()];
        }
        // If two feet have no contact, force set double support contact
        if ( !m_contactStates.data[contact_states_index_map["rleg"]] && !m_contactStates.data[contact_states_index_map["lleg"]] ) {
            m_contactStates.data[contact_states_index_map["rleg"]] = true;
            m_contactStates.data[contact_states_index_map["lleg"]] = true;
        }
    }
    for ( std::map<std::string, ABCIKparam>::iterator it = ikp.begin(); it != ikp.end(); it++ ) {
        size_t idx = contact_states_index_map[it->first];
        // Set limbCOPOffset
        m_limbCOPOffset[idx].data.x = 0;
        m_limbCOPOffset[idx].data.y = 0;
        m_limbCOPOffset[idx].data.z = 0;
        // Set toe heel ratio is not used while double support
        m_toeheelRatio.data[idx] = rats::no_using_toe_heel_ratio;
    }
};

void AutoBalancer::interpolateLegNamesAndZMPOffsets()
{
    if (!zmp_offset_interpolator->isEmpty()) {
      double *default_zmp_offsets_output = new double[ikp.size()*3];
      zmp_offset_interpolator->get(default_zmp_offsets_output, true);
      for (size_t i = 0; i < ikp.size(); i++)
        for (size_t j = 0; j < 3; j++)
          default_zmp_offsets[i](j) = default_zmp_offsets_output[i*3+j];
      delete[] default_zmp_offsets_output;
      if (DEBUGP) {
        std::cerr << "[" << m_profile.instance_name << "] default_zmp_offsets (interpolated)" << std::endl;
        for ( std::map<std::string, ABCIKparam>::iterator it = ikp.begin(); it != ikp.end(); it++ ) {
            std::cerr << "[" << m_profile.instance_name << "]   " << it->first << " = " << default_zmp_offsets[contact_states_index_map[it->first]].format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "    [", "]")) << "[m]" << std::endl;
        }
      }
    }
    if (!leg_names_interpolator->isEmpty()) {
        leg_names_interpolator->get(&leg_names_interpolator_ratio, true);
    }else {
        leg_names_interpolator_ratio = 1.0;
    }
};

void AutoBalancer::calcFixCoordsForAdjustFootstep (coordinates& tmp_fix_coords)
{
    double tmp = 0.0;
    adjust_footstep_interpolator->get(&tmp, true);
    //std::cerr << "[" << m_profile.instance_name << "] adjust ratio " << tmp << std::endl;
    ikp["rleg"].target_p0 = (1-tmp) * ikp["rleg"].adjust_interpolation_org_p0 + tmp*ikp["rleg"].adjust_interpolation_target_p0;
    ikp["lleg"].target_p0 = (1-tmp) * ikp["lleg"].adjust_interpolation_org_p0 + tmp*ikp["lleg"].adjust_interpolation_target_p0;
    rats::mid_rot(ikp["rleg"].target_r0, tmp, ikp["rleg"].adjust_interpolation_org_r0, ikp["rleg"].adjust_interpolation_target_r0);
    rats::mid_rot(ikp["lleg"].target_r0, tmp, ikp["lleg"].adjust_interpolation_org_r0, ikp["lleg"].adjust_interpolation_target_r0);
    coordinates tmprc, tmplc;
    tmprc.pos = ikp["rleg"].target_p0;
    tmprc.rot = ikp["rleg"].target_r0;
    tmplc.pos = ikp["lleg"].target_p0;
    tmplc.rot = ikp["lleg"].target_r0;
    rats::mid_coords(tmp_fix_coords, 0.5, tmprc, tmplc);
    //fix_leg_coords = tmp_fix_coords;
};

void AutoBalancer::rotateRefForcesForFixCoords (coordinates& tmp_fix_coords)
{
    /* update ref_forces ;; StateHolder's absolute -> AutoBalancer's absolute */
    for (size_t i = 0; i < m_ref_forceIn.size(); i++) {
      // hrp::Matrix33 eeR;
      // hrp::Link* parentlink;
      // hrp::ForceSensor* sensor = m_robot->sensor<hrp::ForceSensor>(sensor_names[i]);
      // if (sensor) parentlink = sensor->link;
      // else parentlink = m_vfs[sensor_names[i]].link;
      // for ( std::map<std::string, ABCIKparam>::iterator it = ikp.begin(); it != ikp.end(); it++ ) {
      //     if (it->second.target_link->name == parentlink->name) eeR = parentlink->R * it->second.localR;
      // }
      // End effector frame
      //ref_forces[i] = eeR * hrp::Vector3(m_ref_force[i].data[0], m_ref_force[i].data[1], m_ref_force[i].data[2]);
      // world frame
      ref_forces[i] = tmp_fix_coords.rot * hrp::Vector3(m_ref_force[i].data[0], m_ref_force[i].data[1], m_ref_force[i].data[2]);
    }
    sbp_offset = tmp_fix_coords.rot * hrp::Vector3(sbp_offset);
};

void AutoBalancer::updateTargetCoordsForHandFixMode (coordinates& tmp_fix_coords)
{
    // Move hand for hand fix mode
    //   If arms' ABCIKparam.is_active is true, move hands according to cog velocity.
    //   If is_hand_fix_mode is false, no hand fix mode and move hands according to cog velocity.
    //   If is_hand_fix_mode is true, hand fix mode and do not move hands in Y axis in tmp_fix_coords.rot.    
    if (gg_is_walking) {
        // hand control while walking = solve hand ik with is_hand_fix_mode and solve hand ik without is_hand_fix_mode
        bool is_hand_control_while_walking = false;
        for ( std::map<std::string, ABCIKparam>::iterator it = ikp.begin(); it != ikp.end(); it++ ) {
            if ( it->second.is_active && std::find(leg_names.begin(), leg_names.end(), it->first) == leg_names.end()
                 && it->first.find("arm") != std::string::npos ) {
                is_hand_control_while_walking = true;
            }
        }
        if (is_hand_control_while_walking) {
        //if (false) { // Disabled temporarily
            // Store hand_fix_initial_offset in the initialization of walking
            if (is_hand_fix_initial) hand_fix_initial_offset = tmp_fix_coords.rot.transpose() * (hrp::Vector3(gg->get_cog()(0), gg->get_cog()(1), tmp_fix_coords.pos(2)) - tmp_fix_coords.pos);
            is_hand_fix_initial = false;
            hrp::Vector3 dif_p = hrp::Vector3(gg->get_cog()(0), gg->get_cog()(1), tmp_fix_coords.pos(2)) - tmp_fix_coords.pos - tmp_fix_coords.rot * hand_fix_initial_offset;
            if (is_hand_fix_mode) {
                dif_p = tmp_fix_coords.rot.transpose() * dif_p;
                dif_p(1) = 0;
                dif_p = tmp_fix_coords.rot * dif_p;
            }
            for ( std::map<std::string, ABCIKparam>::iterator it = ikp.begin(); it != ikp.end(); it++ ) {
                if ( it->second.is_active && std::find(leg_names.begin(), leg_names.end(), it->first) == leg_names.end()
                     && it->first.find("arm") != std::string::npos ) {
                    it->second.target_p0 = it->second.target_p0 + dif_p;
                }
            }
        }
    }
};

void AutoBalancer::calculateOutputRefForces ()
{
    // TODO : need to be updated for multicontact and other walking
    if (leg_names.size() == 2) {
        std::vector<hrp::Vector3> ee_pos;
        for (size_t i = 0 ; i < leg_names.size(); i++) {
            ABCIKparam& tmpikp = ikp[leg_names[i]];
            ee_pos.push_back(tmpikp.target_p0 + tmpikp.target_r0 * default_zmp_offsets[i]);
        }
        double alpha = (ref_zmp - ee_pos[1]).norm() / ((ee_pos[0] - ref_zmp).norm() + (ee_pos[1] - ref_zmp).norm());
        if (alpha>1.0) alpha = 1.0;
        if (alpha<0.0) alpha = 0.0;
        if (DEBUGP) {
            std::cerr << "[" << m_profile.instance_name << "] alpha:" << alpha << std::endl;
        }
        double mg = m_robot->totalMass() * gg->get_gravitational_acceleration();
        m_force[0].data[2] = alpha * mg;
        m_force[1].data[2] = (1-alpha) * mg;
    }
    if ( use_force == MODE_REF_FORCE_WITH_FOOT || use_force == MODE_REF_FORCE_RFU_EXT_MOMENT ) { // TODO : use other use_force mode. This should be depends on Stabilizer distribution mode.
        distributeReferenceZMPToWrenches (ref_zmp);
    }
    prev_ref_zmp = ref_zmp;
};

hrp::Vector3 AutoBalancer::calcFootMidPosUsingZMPWeightMap ()
{
    hrp::Vector3 tmp_foot_mid_pos(hrp::Vector3::Zero());
    std::map<leg_type, std::string> leg_type_map = gg->get_leg_type_map();
    std::map<leg_type, double> zmp_weight_map = gg->get_zmp_weight_map();
    double sum_of_weight = 0.0;
    for (size_t i = 0; i < leg_names.size(); i++) {
        ABCIKparam& tmpikp = ikp[leg_names[i]];
        // for foot_mid_pos
        std::map<leg_type, std::string>::const_iterator dst = std::find_if(leg_type_map.begin(), leg_type_map.end(), (&boost::lambda::_1->* &std::map<leg_type, std::string>::value_type::second == leg_names[i]));
        tmp_foot_mid_pos += (tmpikp.target_p0 + tmpikp.target_r0 * default_zmp_offsets[i]) * zmp_weight_map[dst->first];
        sum_of_weight += zmp_weight_map[dst->first];
    }
    tmp_foot_mid_pos *= (1.0 / sum_of_weight);
    return tmp_foot_mid_pos;
};

void AutoBalancer::updateWalkingVelocityFromHandError (coordinates& tmp_fix_coords)
{
    // TODO : check frame and robot state for this calculation
    if ( gg_is_walking && gg->get_lcg_count() == gg->get_overwrite_check_timing()+2 ) {
        hrp::Vector3 vel_htc(calc_vel_from_hand_error(tmp_fix_coords));
        gg->set_offset_velocity_param(vel_htc(0), vel_htc(1) ,vel_htc(2));
    }//  else {
    //     if ( gg_is_walking && gg->get_lcg_count() == static_cast<size_t>(gg->get_default_step_time()/(2*m_dt))-1) {
    //         gg->set_offset_velocity_param(0,0,0);
    //     }
    // }
};

void AutoBalancer::calcReferenceJointAnglesForIK ()
{
    fik->overwrite_ref_ja_index_vec.clear();
    // Fix for toe joint
    for ( std::map<std::string, ABCIKparam>::iterator it = ikp.begin(); it != ikp.end(); it++ ) {
        if (it->second.is_active && it->second.has_toe_joint && gg->get_use_toe_joint()) {
            int i = it->second.target_link->jointId;
            if (gg->get_swing_leg_names().front() == it->first) {
                fik->qrefv[i] = fik->qrefv[i] + -1 * gg->get_foot_dif_rot_angle();
            }
            fik->overwrite_ref_ja_index_vec.push_back(i);
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
      if (leg_names[i].find("leg") != std::string::npos) foot_coords.push_back(coordinates((tmpikp.target_link->p + tmpikp.target_link->R * tmpikp.localPos),
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

void AutoBalancer::fixLegToCoords2 (coordinates& tmp_fix_coords)
{
    // Tempolarily modify tmp_fix_coords
    // This will be removed after seq outputs adequate waistRPY discussed in https://github.com/fkanehiro/hrpsys-base/issues/272
    // Snap input tmp_fix_coords to XY plan projection.
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
}

void AutoBalancer::solveFullbodyIK ()
{
  // Set ik target params
  fik->target_root_p = target_root_p;
  fik->target_root_R = target_root_R;
  for ( std::map<std::string, SimpleFullbodyInverseKinematicsSolver::IKparam>::iterator it = fik->ikp.begin(); it != fik->ikp.end(); it++ ) {
      it->second.target_p0 = ikp[it->first].target_p0;
      it->second.target_r0 = ikp[it->first].target_r0;
  }
  fik->ratio_for_vel = transition_interpolator_ratio * leg_names_interpolator_ratio;
//  fik->current_tm = m_qRef.tm;
  for ( std::map<std::string, ABCIKparam>::iterator it = ikp.begin(); it != ikp.end(); it++ ) {
      fik->ikp[it->first].is_ik_enable = it->second.is_active;
  }
  // Revert
  fik->revertRobotStateToCurrent();
  // TODO : SBP calculation is outside of solve ik?
  hrp::Vector3 tmp_input_sbp = hrp::Vector3(0,0,0);
  static_balance_point_proc_one(tmp_input_sbp, ref_zmp(2));
  hrp::Vector3 dif_cog = tmp_input_sbp - ref_cog;
  // Solve IK
  fik->solveFullbodyIK (dif_cog, transition_interpolator->isEmpty());
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
  transition_interpolator->setGoal(&tmp_ratio, transition_time, true);
  prev_ref_zmp = ref_zmp;
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
  transition_interpolator->setGoal(&tmp_ratio, transition_time, true);
  control_mode = MODE_SYNC_TO_IDLE;
}

bool AutoBalancer::startWalking ()
{
  if ( control_mode != MODE_ABC ) {
    std::cerr << "[" << m_profile.instance_name << "] Cannot start walking without MODE_ABC. Please startAutoBalancer." << std::endl;
    return false;
  }
  {
    Guard guard(m_mutex);
    fik->resetIKFailParam();
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
        init_support_leg_steps.push_back(step_node(*it, coordinates(ikp[*it].target_p0, ikp[*it].target_r0), 0, 0, 0, 0));
    for (std::vector<std::string>::iterator it = init_swing_leg_names.begin(); it != init_swing_leg_names.end(); it++)
        init_swing_leg_dst_steps.push_back(step_node(*it, coordinates(ikp[*it].target_p0, ikp[*it].target_r0), 0, 0, 0, 0));
    gg->set_default_zmp_offsets(default_zmp_offsets);
    gg->initialize_gait_parameter(ref_cog, init_support_leg_steps, init_swing_leg_dst_steps);
  }
  is_hand_fix_initial = true;
  while ( !gg->proc_one_tick() );
  {
    Guard guard(m_mutex);
    gg_is_walking = gg_solved = true;
  }
  return true;
}

void AutoBalancer::stopWalking ()
{
  std::vector<coordinates> tmp_end_coords_list;
  for (std::vector<string>::iterator it = leg_names.begin(); it != leg_names.end(); it++) {
      if ((*it).find("leg") != std::string::npos) tmp_end_coords_list.push_back(coordinates(ikp[*it].target_p0, ikp[*it].target_r0));
  }
  multi_mid_coords(fix_leg_coords, tmp_end_coords_list);
  fixLegToCoords(fix_leg_coords.pos, fix_leg_coords.rot);
  gg->clear_footstep_nodes_list();
  gg_is_walking = false;
}

bool AutoBalancer::startAutoBalancer (const OpenHRP::AutoBalancerService::StrSequence& limbs)
{
  if (control_mode == MODE_IDLE) {
    fik->resetIKFailParam();
    startABCparam(limbs);
    waitABCTransition();
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
    //  if ( !gg_is_walking && !is_stop_mode) {
  if ( !is_stop_mode) {
    gg->set_all_limbs(leg_names);
    coordinates start_ref_coords;
    std::vector<coordinates> initial_support_legs_coords;
    std::vector<leg_type> initial_support_legs;
    bool is_valid_gait_type = calc_inital_support_legs(y, initial_support_legs_coords, initial_support_legs, start_ref_coords);
    if (is_valid_gait_type == false) return false;
    bool ret = gg->go_pos_param_2_footstep_nodes_list(x, y, th,
                                                      initial_support_legs_coords, // Dummy if gg_is_walking
                                                      start_ref_coords,            // Dummy if gg_is_walking
                                                      initial_support_legs,        // Dummy if gg_is_walking
                                                      (!gg_is_walking)); // If gg_is_walking, initialize. Otherwise, not initialize and overwrite footsteps.
    if (!ret) {
        std::cerr << "[" << m_profile.instance_name << "] Cannot goPos because of invalid timing." << std::endl;
    }
    if ( !gg_is_walking ) { // Initializing
        ret = startWalking();
    }
    return ret;
  } else {
    std::cerr << "[" << m_profile.instance_name << "] Cannot goPos while stopping mode." << std::endl;
    return false;
  }
}

bool AutoBalancer::goVelocity(const double& vx, const double& vy, const double& vth)
{
  gg->set_all_limbs(leg_names);
  bool ret = true;
  if (gg_is_walking && gg_solved) {
    gg->set_velocity_param(vx, vy, vth);
  } else {
    coordinates ref_coords;
    ref_coords.pos = (ikp["rleg"].target_p0+ikp["lleg"].target_p0)*0.5;
    mid_rot(ref_coords.rot, 0.5, ikp["rleg"].target_r0, ikp["lleg"].target_r0);
    std::vector<leg_type> current_legs;
    switch(gait_type) {
    case BIPED:
        current_legs.assign (1, vy > 0 ? RLEG : LLEG);
        break;
    case TROT:
        current_legs = (vy > 0 ? boost::assign::list_of(RLEG)(LARM) : boost::assign::list_of(LLEG)(RARM)).convert_to_container < std::vector<leg_type> > ();
        break;
    case PACE:
        current_legs = (vy > 0 ? boost::assign::list_of(RLEG)(RARM) : boost::assign::list_of(LLEG)(LARM)).convert_to_container < std::vector<leg_type> > ();
        break;
    case CRAWL:
        std::cerr << "[" << m_profile.instance_name << "] crawl walk[" << gait_type << "] is not implemented yet." << std::endl;
        return false;
    case GALLOP:
        /* at least one leg shoud be in contact */
        std::cerr << "[" << m_profile.instance_name << "] gallop walk[" << gait_type << "] is not implemented yet." << std::endl;
        return false;
    default: break;
    }
    gg->initialize_velocity_mode(ref_coords, vx, vy, vth, current_legs);
    ret = startWalking();
  }
  return ret;
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
          spss[i].sps[j].toe_angle = ((!gg_is_walking && i==0) ? 0.0 : gg->get_toe_angle());
          spss[i].sps[j].heel_angle = ((!gg_is_walking && i==0) ? 0.0 : gg->get_heel_angle());
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
                    ABCIKparam& tmpikp = ikp[std::string(fss[0].fs[i].leg)];
                    initial_support_steps.push_back(step_node(std::string(fss[0].fs[i].leg), coordinates(tmpikp.target_p0, tmpikp.target_r0), 0, 0, 0, 0));
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
                for (size_t j = 0; j < fs_vec_list.at(i).size(); j++) {
                    tmp_fns.push_back(step_node(leg_name_vec_list[i][j], fs_vec_list[i][j], spss[i].sps[j].step_height, spss[i].sps[j].step_time, spss[i].sps[j].toe_angle, spss[i].sps[j].heel_angle));
                }
                fnsl.push_back(tmp_fns);
            }
        }
        bool ret = true;
        if (gg_is_walking) {
            std::cerr << "[" << m_profile.instance_name << "]  Set overwrite footsteps" << std::endl;
            gg->set_overwrite_foot_steps_list(fnsl);
            gg->set_overwrite_foot_step_index(overwrite_fs_idx);
        } else {
            std::cerr << "[" << m_profile.instance_name << "]  Set normal footsteps" << std::endl;
            gg->set_foot_steps_list(fnsl);
            ret = startWalking();
        }
        return ret;
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
  if (i_param.stride_parameter.length() == 4) { // Support old stride_parameter definitions
      gg->set_stride_parameters(i_param.stride_parameter[0], i_param.stride_parameter[1], i_param.stride_parameter[2], i_param.stride_parameter[3],
                                i_param.stride_parameter[1]*0.5, i_param.stride_parameter[2]*0.5);
  } else {
      gg->set_stride_parameters(i_param.stride_parameter[0], i_param.stride_parameter[1], i_param.stride_parameter[2], i_param.stride_parameter[3],
                                i_param.stride_parameter[4], i_param.stride_parameter[5]);
  }
  std::vector<hrp::Vector3> off;
  for (size_t i = 0; i < i_param.leg_default_translate_pos.length(); i++) {
      off.push_back(hrp::Vector3(i_param.leg_default_translate_pos[i][0], i_param.leg_default_translate_pos[i][1], i_param.leg_default_translate_pos[i][2]));
  }
  gg->set_leg_default_translate_pos(off);
  gg->set_default_step_time(i_param.default_step_time);
  gg->set_default_step_height(i_param.default_step_height);
  gg->set_default_double_support_ratio_before(i_param.default_double_support_ratio/2.0);
  gg->set_default_double_support_ratio_after(i_param.default_double_support_ratio/2.0);
  gg->set_default_double_support_static_ratio_before(i_param.default_double_support_static_ratio/2.0);
  gg->set_default_double_support_static_ratio_after(i_param.default_double_support_static_ratio/2.0);
  gg->set_default_double_support_ratio_swing_before(i_param.default_double_support_ratio/2.0);
  gg->set_default_double_support_ratio_swing_after(i_param.default_double_support_ratio/2.0);
  // gg->set_default_double_support_ratio_before(i_param.default_double_support_ratio_before);
  // gg->set_default_double_support_ratio_after(i_param.default_double_support_ratio_after);
  // gg->set_default_double_support_static_ratio_before(i_param.default_double_support_static_ratio_before);
  // gg->set_default_double_support_static_ratio_after(i_param.default_double_support_static_ratio_after);
  // gg->set_default_double_support_ratio_swing_before(i_param.default_double_support_ratio_before);
  // gg->set_default_double_support_ratio_swing_after(i_param.default_double_support_ratio_after);
  // gg->set_default_double_support_ratio_swing_before(i_param.default_double_support_ratio_swing_before);
  // gg->set_default_double_support_ratio_swing_after(i_param.default_double_support_ratio_swing_after);
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
  } else if (i_param.default_orbit_type == OpenHRP::AutoBalancerService::CROSS) {
    gg->set_default_orbit_type(CROSS);
  }
  gg->set_swing_trajectory_delay_time_offset(i_param.swing_trajectory_delay_time_offset);
  gg->set_swing_trajectory_final_distance_weight(i_param.swing_trajectory_final_distance_weight);
  gg->set_swing_trajectory_time_offset_xy2z(i_param.swing_trajectory_time_offset_xy2z);
  gg->set_stair_trajectory_way_point_offset(hrp::Vector3(i_param.stair_trajectory_way_point_offset[0], i_param.stair_trajectory_way_point_offset[1], i_param.stair_trajectory_way_point_offset[2]));
  gg->set_cycloid_delay_kick_point_offset(hrp::Vector3(i_param.cycloid_delay_kick_point_offset[0], i_param.cycloid_delay_kick_point_offset[1], i_param.cycloid_delay_kick_point_offset[2]));  
  gg->set_gravitational_acceleration(i_param.gravitational_acceleration);
  gg->set_toe_angle(i_param.toe_angle);
  gg->set_heel_angle(i_param.heel_angle);
  gg->set_toe_pos_offset_x(i_param.toe_pos_offset_x);
  gg->set_heel_pos_offset_x(i_param.heel_pos_offset_x);
  gg->set_toe_zmp_offset_x(i_param.toe_zmp_offset_x);
  gg->set_heel_zmp_offset_x(i_param.heel_zmp_offset_x);
  gg->set_toe_check_thre(i_param.toe_check_thre);
  gg->set_heel_check_thre(i_param.heel_check_thre);
  std::vector<double> tmp_ratio(i_param.toe_heel_phase_ratio.get_buffer(), i_param.toe_heel_phase_ratio.get_buffer()+i_param.toe_heel_phase_ratio.length());
  std::cerr << "[" << m_profile.instance_name << "]   "; // for set_toe_heel_phase_ratio
  gg->set_toe_heel_phase_ratio(tmp_ratio);
  gg->set_use_toe_joint(i_param.use_toe_joint);
  gg->set_use_toe_heel_transition(i_param.use_toe_heel_transition);
  gg->set_use_toe_heel_auto_set(i_param.use_toe_heel_auto_set);
  gg->set_zmp_weight_map(boost::assign::map_list_of<leg_type, double>(RLEG, i_param.zmp_weight_map[0])(LLEG, i_param.zmp_weight_map[1])(RARM, i_param.zmp_weight_map[2])(LARM, i_param.zmp_weight_map[3]));
  gg->set_optional_go_pos_finalize_footstep_num(i_param.optional_go_pos_finalize_footstep_num);
  gg->set_overwritable_footstep_index_offset(i_param.overwritable_footstep_index_offset);
  gg->set_leg_margin(i_param.leg_margin);
  gg->set_stride_limitation_for_circle_type(i_param.stride_limitation_for_circle_type);
  gg->set_overwritable_stride_limitation(i_param.overwritable_stride_limitation);
  gg->set_use_stride_limitation(i_param.use_stride_limitation);
  gg->set_footstep_modification_gain(i_param.footstep_modification_gain);
  gg->set_modify_footsteps(i_param.modify_footsteps);
  gg->set_cp_check_margin(i_param.cp_check_margin);
  gg->set_margin_time_ratio(i_param.margin_time_ratio);
  if (i_param.stride_limitation_type == OpenHRP::AutoBalancerService::SQUARE) {
    gg->set_stride_limitation_type(SQUARE);
  } else if (i_param.stride_limitation_type == OpenHRP::AutoBalancerService::CIRCLE) {
    gg->set_stride_limitation_type(CIRCLE);
  }

  // print
  gg->print_param(std::string(m_profile.instance_name));
  return true;
};

bool AutoBalancer::getGaitGeneratorParam(OpenHRP::AutoBalancerService::GaitGeneratorParam& i_param)
{
  gg->get_stride_parameters(i_param.stride_parameter[0], i_param.stride_parameter[1], i_param.stride_parameter[2], i_param.stride_parameter[3], i_param.stride_parameter[4], i_param.stride_parameter[5]);
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
  i_param.default_double_support_ratio_before = gg->get_default_double_support_ratio_before();
  i_param.default_double_support_ratio_after = gg->get_default_double_support_ratio_after();
  i_param.default_double_support_static_ratio_before = gg->get_default_double_support_static_ratio_before();
  i_param.default_double_support_static_ratio_after = gg->get_default_double_support_static_ratio_after();
  i_param.default_double_support_ratio_swing_before = gg->get_default_double_support_ratio_swing_before();
  i_param.default_double_support_ratio_swing_after = gg->get_default_double_support_ratio_swing_after();
  i_param.default_double_support_ratio = i_param.default_double_support_ratio_before + i_param.default_double_support_ratio_after;
  i_param.default_double_support_static_ratio = i_param.default_double_support_static_ratio_before + i_param.default_double_support_static_ratio_after;
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
  } else if (gg->get_default_orbit_type() == CROSS) {
    i_param.default_orbit_type = OpenHRP::AutoBalancerService::CROSS;
  }

  hrp::Vector3 tmpv = gg->get_stair_trajectory_way_point_offset();
  for (size_t i = 0; i < 3; i++) i_param.stair_trajectory_way_point_offset[i] = tmpv(i);
  tmpv = gg->get_cycloid_delay_kick_point_offset();
  for (size_t i = 0; i < 3; i++) i_param.cycloid_delay_kick_point_offset[i] = tmpv(i);
  i_param.swing_trajectory_delay_time_offset = gg->get_swing_trajectory_delay_time_offset();
  i_param.swing_trajectory_final_distance_weight = gg->get_swing_trajectory_final_distance_weight();
  i_param.swing_trajectory_time_offset_xy2z = gg->get_swing_trajectory_time_offset_xy2z();
  i_param.gravitational_acceleration = gg->get_gravitational_acceleration();
  i_param.toe_angle = gg->get_toe_angle();
  i_param.heel_angle = gg->get_heel_angle();
  i_param.toe_pos_offset_x = gg->get_toe_pos_offset_x();
  i_param.heel_pos_offset_x = gg->get_heel_pos_offset_x();
  i_param.toe_zmp_offset_x = gg->get_toe_zmp_offset_x();
  i_param.heel_zmp_offset_x = gg->get_heel_zmp_offset_x();
  i_param.toe_check_thre = gg->get_toe_check_thre();
  i_param.heel_check_thre = gg->get_heel_check_thre();
  std::vector<double> ratio(gg->get_NUM_TH_PHASES(),0.0);
  gg->get_toe_heel_phase_ratio(ratio);
  for (int i = 0; i < gg->get_NUM_TH_PHASES(); i++) i_param.toe_heel_phase_ratio[i] = ratio[i];
  i_param.use_toe_joint = gg->get_use_toe_joint();
  i_param.use_toe_heel_transition = gg->get_use_toe_heel_transition();
  i_param.use_toe_heel_auto_set = gg->get_use_toe_heel_auto_set();
  std::map<leg_type, double> tmp_zmp_weight_map = gg->get_zmp_weight_map();
  i_param.zmp_weight_map[0] = tmp_zmp_weight_map[RLEG];
  i_param.zmp_weight_map[1] = tmp_zmp_weight_map[LLEG];
  i_param.zmp_weight_map[2] = tmp_zmp_weight_map[RARM];
  i_param.zmp_weight_map[3] = tmp_zmp_weight_map[LARM];
  i_param.optional_go_pos_finalize_footstep_num = gg->get_optional_go_pos_finalize_footstep_num();
  i_param.overwritable_footstep_index_offset = gg->get_overwritable_footstep_index_offset();
  for (size_t i=0; i<4; i++) {
    i_param.leg_margin[i] = gg->get_leg_margin(i);
  }
  for (size_t i=0; i<5; i++) {
    i_param.stride_limitation_for_circle_type[i] = gg->get_stride_limitation_for_circle_type(i);
  }
  for (size_t i=0; i<5; i++) {
    i_param.overwritable_stride_limitation[i] = gg->get_overwritable_stride_limitation(i);
  }
  i_param.use_stride_limitation = gg->get_use_stride_limitation();
  i_param.footstep_modification_gain = gg->get_footstep_modification_gain();
  i_param.modify_footsteps = gg->get_modify_footsteps();
  for (size_t i=0; i<2; i++) {
    i_param.cp_check_margin[i] = gg->get_cp_check_margin(i);
  }
  i_param.margin_time_ratio = gg->get_margin_time_ratio();
  if (gg->get_stride_limitation_type() == SQUARE) {
    i_param.stride_limitation_type = OpenHRP::AutoBalancerService::SQUARE;
  } else if (gg->get_stride_limitation_type() == CIRCLE) {
    i_param.stride_limitation_type = OpenHRP::AutoBalancerService::CIRCLE;
  }
  return true;
};

bool AutoBalancer::setAutoBalancerParam(const OpenHRP::AutoBalancerService::AutoBalancerParam& i_param)
{
  Guard guard(m_mutex);
  std::cerr << "[" << m_profile.instance_name << "] setAutoBalancerParam" << std::endl;
  double *default_zmp_offsets_array = new double[ikp.size()*3];
  for (size_t i = 0; i < ikp.size(); i++)
    for (size_t j = 0; j < 3; j++)
      default_zmp_offsets_array[i*3+j] = i_param.default_zmp_offsets[i][j];
  zmp_transition_time = i_param.zmp_transition_time;
  adjust_footstep_transition_time = i_param.adjust_footstep_transition_time;
  if (zmp_offset_interpolator->isEmpty()) {
      zmp_offset_interpolator->clear();
      zmp_offset_interpolator->setGoal(default_zmp_offsets_array, zmp_transition_time, true);
  } else {
      std::cerr << "[" << m_profile.instance_name << "]   default_zmp_offsets cannot be set because interpolating." << std::endl;
  }
  if (control_mode == MODE_IDLE) {
    switch (i_param.use_force_mode) {
    case OpenHRP::AutoBalancerService::MODE_NO_FORCE:
        use_force = MODE_NO_FORCE;
        break;
    case OpenHRP::AutoBalancerService::MODE_REF_FORCE:
        use_force = MODE_REF_FORCE;
        break;
    case OpenHRP::AutoBalancerService::MODE_REF_FORCE_WITH_FOOT:
        use_force = MODE_REF_FORCE_WITH_FOOT;
        break;
    case OpenHRP::AutoBalancerService::MODE_REF_FORCE_RFU_EXT_MOMENT:
        use_force = MODE_REF_FORCE_RFU_EXT_MOMENT;
        break;
    default:
        break;
    }
  } else {
      std::cerr << "[" << m_profile.instance_name << "]   use_force_mode cannot be changed to [" << i_param.use_force_mode << "] during MODE_ABC, MODE_SYNC_TO_IDLE or MODE_SYNC_TO_ABC." << std::endl;
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
  std::vector<std::string> cur_leg_names, dst_leg_names;
  cur_leg_names = leg_names;
  for (size_t i = 0; i < i_param.leg_names.length(); i++) {
      dst_leg_names.push_back(std::string(i_param.leg_names[i]));
  }
  std::sort(cur_leg_names.begin(), cur_leg_names.end());
  std::sort(dst_leg_names.begin(), dst_leg_names.end());
  if (cur_leg_names != dst_leg_names) {
      if (leg_names_interpolator->isEmpty()) {
          leg_names.clear();
          leg_names = dst_leg_names;
          if (control_mode == MODE_ABC) {
              double tmp_ratio = 0.0;
              leg_names_interpolator->set(&tmp_ratio);
              tmp_ratio = 1.0;
              leg_names_interpolator->setGoal(&tmp_ratio, 5.0, true);
              control_mode = MODE_SYNC_TO_ABC;
          }
      }
  } else {
      std::cerr << "[" << m_profile.instance_name << "]   leg_names cannot be set because interpolating." << std::endl;
  }
  if (!gg_is_walking) {
      is_hand_fix_mode = i_param.is_hand_fix_mode;
      std::cerr << "[" << m_profile.instance_name << "]   is_hand_fix_mode = " << is_hand_fix_mode << std::endl;
  } else {
      std::cerr << "[" << m_profile.instance_name << "]   is_hand_fix_mode cannot be set in (gg_is_walking = true). Current is_hand_fix_mode is " << (is_hand_fix_mode?"true":"false") << std::endl;
  }
  if (control_mode == MODE_IDLE) {
      for (size_t i = 0; i < i_param.end_effector_list.length(); i++) {
          std::map<std::string, ABCIKparam>::iterator it = ikp.find(std::string(i_param.end_effector_list[i].leg));
          memcpy(it->second.localPos.data(), i_param.end_effector_list[i].pos, sizeof(double)*3);
          it->second.localR = (Eigen::Quaternion<double>(i_param.end_effector_list[i].rot[0], i_param.end_effector_list[i].rot[1], i_param.end_effector_list[i].rot[2], i_param.end_effector_list[i].rot[3])).normalized().toRotationMatrix();
      }
  } else {
      std::cerr << "[" << m_profile.instance_name << "] cannot change end-effectors except during MODE_IDLE" << std::endl;
  }
  if (i_param.default_gait_type == OpenHRP::AutoBalancerService::BIPED) {
      gait_type = BIPED;
  } else if (i_param.default_gait_type == OpenHRP::AutoBalancerService::TROT) {
      gait_type = TROT;
  } else if (i_param.default_gait_type == OpenHRP::AutoBalancerService::PACE) {
      gait_type = PACE;
  } else if (i_param.default_gait_type == OpenHRP::AutoBalancerService::CRAWL) {
      gait_type = CRAWL;
  } else if (i_param.default_gait_type == OpenHRP::AutoBalancerService::GALLOP) {
      gait_type = GALLOP;
  }
  // Ref force balancing
  std::cerr << "[" << m_profile.instance_name << "] Ref force balancing" << std::endl;
  if ( use_force == MODE_REF_FORCE_WITH_FOOT && control_mode != MODE_IDLE ) {
      std::cerr << "[" << m_profile.instance_name << "]   additional_force_applied_point_offset and additional_force_applied_link_name cannot be updated during MODE_REF_FORCE_WITH_FOOT and non-MODE_IDLE"<< std::endl;
  } else if ( !m_robot->link(std::string(i_param.additional_force_applied_link_name)) ) {
      std::cerr << "[" << m_profile.instance_name << "]   Invalid link name for additional_force_applied_link_name = " << i_param.additional_force_applied_link_name << std::endl;
  } else {
      additional_force_applied_link = m_robot->link(std::string(i_param.additional_force_applied_link_name));
      for (size_t i = 0; i < 3; i++) {
          additional_force_applied_point_offset(i) = i_param.additional_force_applied_point_offset[i];
      }
      std::cerr << "[" << m_profile.instance_name << "]   Link name for additional_force_applied_link_name = " << additional_force_applied_link->name << ", additional_force_applied_point_offset = " << additional_force_applied_point_offset.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "    [", "]")) << "[m]" << std::endl;
  }

  for (std::map<std::string, ABCIKparam>::iterator it = ikp.begin(); it != ikp.end(); it++) {
      std::cerr << "[" << m_profile.instance_name << "] End Effector [" << it->first << "]" << std::endl;
      std::cerr << "[" << m_profile.instance_name << "]   localpos = " << it->second.localPos.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "    [", "]")) << "[m]" << std::endl;
      std::cerr << "[" << m_profile.instance_name << "]   localR = " << it->second.localR.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", "\n", "    [", "]")) << std::endl;
  }

  std::cerr << "[" << m_profile.instance_name << "]   default_zmp_offsets = ";
  for (size_t i = 0; i < ikp.size() * 3; i++) {
      std::cerr << default_zmp_offsets_array[i] << " ";
  }
  std::cerr << std::endl;
  delete[] default_zmp_offsets_array;
  std::cerr << "[" << m_profile.instance_name << "]   use_force_mode = " << getUseForceModeString() << std::endl;
  std::cerr << "[" << m_profile.instance_name << "]   graspless_manip_mode = " << graspless_manip_mode << std::endl;
  std::cerr << "[" << m_profile.instance_name << "]   graspless_manip_arm = " << graspless_manip_arm << std::endl;
  std::cerr << "[" << m_profile.instance_name << "]   graspless_manip_p_gain = " << graspless_manip_p_gain.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "    [", "]")) << std::endl;
  std::cerr << "[" << m_profile.instance_name << "]   graspless_manip_reference_trans_pos = " << graspless_manip_reference_trans_coords.pos.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "    [", "]")) << std::endl;
  std::cerr << "[" << m_profile.instance_name << "]   graspless_manip_reference_trans_rot = " << graspless_manip_reference_trans_coords.rot.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", "\n", "    [", "]")) << std::endl;
  std::cerr << "[" << m_profile.instance_name << "]   transition_time = " << transition_time << "[s], zmp_transition_time = " << zmp_transition_time << "[s], adjust_footstep_transition_time = " << adjust_footstep_transition_time << "[s]" << std::endl;
  for (std::vector<std::string>::iterator it = leg_names.begin(); it != leg_names.end(); it++) std::cerr << "[" << m_profile.instance_name << "]   leg_names [" << *it << "]" << std::endl;
  std::cerr << "[" << m_profile.instance_name << "]   default_gait_type = " << gait_type << std::endl;
  // FIK
  fik->move_base_gain = i_param.move_base_gain;
  fik->pos_ik_thre = i_param.pos_ik_thre;
  fik->rot_ik_thre = i_param.rot_ik_thre;
  fik->printParam();
  // IK limb parameters
  fik->setIKParam(ee_vec, i_param.ik_limb_parameters);
  // Limb stretch avoidance
  fik->use_limb_stretch_avoidance = i_param.use_limb_stretch_avoidance;
  fik->limb_stretch_avoidance_time_const = i_param.limb_stretch_avoidance_time_const;
  for (size_t i = 0; i < 2; i++) {
    fik->limb_stretch_avoidance_vlimit[i] = i_param.limb_stretch_avoidance_vlimit[i];
  }
  for (size_t i = 0; i < fik->ikp.size(); i++) {
    fik->ikp[ee_vec[i]].limb_length_margin = i_param.limb_length_margin[i];
  }
  return true;
};

bool AutoBalancer::getAutoBalancerParam(OpenHRP::AutoBalancerService::AutoBalancerParam& i_param)
{
  i_param.default_zmp_offsets.length(ikp.size());
  for (size_t i = 0; i < ikp.size(); i++) {
      i_param.default_zmp_offsets[i].length(3);
      for (size_t j = 0; j < 3; j++) {
          i_param.default_zmp_offsets[i][j] = default_zmp_offsets[i](j);
      }
  }
  switch(control_mode) {
  case MODE_IDLE: i_param.controller_mode = OpenHRP::AutoBalancerService::MODE_IDLE; break;
  case MODE_ABC: i_param.controller_mode = OpenHRP::AutoBalancerService::MODE_ABC; break;
  case MODE_SYNC_TO_IDLE: i_param.controller_mode = OpenHRP::AutoBalancerService::MODE_SYNC_TO_IDLE; break;
  case MODE_SYNC_TO_ABC: i_param.controller_mode = OpenHRP::AutoBalancerService::MODE_SYNC_TO_ABC; break;
  default: break;
  }
  switch(use_force) {
  case MODE_NO_FORCE: i_param.use_force_mode = OpenHRP::AutoBalancerService::MODE_NO_FORCE; break;
  case MODE_REF_FORCE: i_param.use_force_mode = OpenHRP::AutoBalancerService::MODE_REF_FORCE; break;
  case MODE_REF_FORCE_WITH_FOOT: i_param.use_force_mode = OpenHRP::AutoBalancerService::MODE_REF_FORCE_WITH_FOOT; break;
  case MODE_REF_FORCE_RFU_EXT_MOMENT: i_param.use_force_mode = OpenHRP::AutoBalancerService::MODE_REF_FORCE_RFU_EXT_MOMENT; break;
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
  i_param.is_hand_fix_mode = is_hand_fix_mode;
  i_param.end_effector_list.length(ikp.size());
  {
      size_t i = 0;
      for (std::map<std::string, ABCIKparam>::const_iterator it = ikp.begin(); it != ikp.end(); it++) {
          copyRatscoords2Footstep(i_param.end_effector_list[i],
                                  coordinates(it->second.localPos, it->second.localR));
          i_param.end_effector_list[i].leg = it->first.c_str();
          i++;
      }
  }
  switch(gait_type) {
  case BIPED:  i_param.default_gait_type = OpenHRP::AutoBalancerService::BIPED;  break;
  case TROT:   i_param.default_gait_type = OpenHRP::AutoBalancerService::TROT;   break;
  case PACE:   i_param.default_gait_type = OpenHRP::AutoBalancerService::PACE;   break;
  case CRAWL:  i_param.default_gait_type = OpenHRP::AutoBalancerService::CRAWL;  break;
  case GALLOP: i_param.default_gait_type = OpenHRP::AutoBalancerService::GALLOP; break;
  default: break;
  }
  // FIK
  i_param.move_base_gain = fik->move_base_gain;
  i_param.pos_ik_thre = fik->pos_ik_thre;
  i_param.rot_ik_thre = fik->rot_ik_thre;
  // IK limb parameters
  fik->getIKParam(ee_vec, i_param.ik_limb_parameters);
  // Limb stretch avoidance
  i_param.use_limb_stretch_avoidance = fik->use_limb_stretch_avoidance;
  i_param.limb_stretch_avoidance_time_const = fik->limb_stretch_avoidance_time_const;
  i_param.limb_length_margin.length(fik->ikp.size());
  for (size_t i = 0; i < 2; i++) {
    i_param.limb_stretch_avoidance_vlimit[i] = fik->limb_stretch_avoidance_vlimit[i];
  }
  for (size_t i = 0; i < ikp.size(); i++) {
    i_param.limb_length_margin[i] = fik->ikp[ee_vec[i]].limb_length_margin;
  }
  i_param.additional_force_applied_link_name = additional_force_applied_link->name.c_str();
  for (size_t i = 0; i < 3; i++) {
      i_param.additional_force_applied_point_offset[i] = additional_force_applied_point_offset(i);
  }
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
  std::cerr << "[" << m_profile.instance_name << "] [" << m_qRef.tm
            << "] adjustFootSteps" << std::endl;
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
      ikp["rleg"].adjust_interpolation_target_r0 = eerot;
      ikp["rleg"].adjust_interpolation_target_p0 = eepos;
      memcpy(eepos.data(), lfootstep.pos, sizeof(double)*3);
      eerot = (Eigen::Quaternion<double>(lfootstep.rot[0], lfootstep.rot[1], lfootstep.rot[2], lfootstep.rot[3])).normalized().toRotationMatrix(); // rtc: 
      ikp["lleg"].adjust_interpolation_target_r0 = eerot;
      ikp["lleg"].adjust_interpolation_target_p0 = eepos;
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
      adjust_footstep_interpolator->setGoal(&tmp, adjust_footstep_transition_time, true);
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

bool AutoBalancer::getGoPosFootstepsSequence(const double& x, const double& y, const double& th, OpenHRP::AutoBalancerService::FootstepsSequence_out o_footstep)
{
    std::cerr << "[" << m_profile.instance_name << "] getGoPosFootstepsSequence" << std::endl;
    o_footstep = new OpenHRP::AutoBalancerService::FootstepsSequence;
    if (gg_is_walking) {
        std::cerr << "[" << m_profile.instance_name << "] Cannot call getGoPosFootstepsSequence in walking" << std::endl;
        return false;
    } else {
        gg->set_all_limbs(leg_names);
        std::vector< std::vector<step_node> > new_footstep_nodes_list;
        coordinates start_ref_coords;
        std::vector<coordinates> initial_support_legs_coords;
        std::vector<leg_type> initial_support_legs;
        bool is_valid_gait_type = calc_inital_support_legs(y, initial_support_legs_coords, initial_support_legs, start_ref_coords);
        if (is_valid_gait_type == false) return false;
        /* go_pos_param_2_footstep_nodes_list_core is const member function  */
        gg->go_pos_param_2_footstep_nodes_list_core (x, y, th,
                                                     initial_support_legs_coords, start_ref_coords, initial_support_legs,
                                                     new_footstep_nodes_list, true, 0);
        o_footstep->length(new_footstep_nodes_list.size());
        for (size_t i = 0; i < new_footstep_nodes_list.size(); i++) {
            o_footstep[i].fs.length(new_footstep_nodes_list.at(i).size());
            for (size_t j = 0; j < new_footstep_nodes_list.at(i).size(); j++) {
                leg_type tmp_leg_type = new_footstep_nodes_list.at(i).at(j).l_r;
                o_footstep[i].fs[j].leg = ((tmp_leg_type == RLEG) ? "rleg":
                                           (tmp_leg_type == LLEG) ? "lleg":
                                           (tmp_leg_type == RARM) ? "rarm":
                                           "larm");
                copyRatscoords2Footstep(o_footstep[i].fs[j], new_footstep_nodes_list.at(i).at(j).worldcoords);
            }
        }
        return true;
    }
};

void AutoBalancer::static_balance_point_proc_one(hrp::Vector3& tmp_input_sbp, const double ref_com_height)
{
  hrp::Vector3 target_sbp = hrp::Vector3(0, 0, 0);
  hrp::Vector3 tmpcog = m_robot->calcCM();
  if ( use_force == MODE_NO_FORCE ) {
    tmp_input_sbp = tmpcog + sbp_cog_offset;
  } else {
    calc_static_balance_point_from_forces(target_sbp, tmpcog, ref_com_height, ref_forces);
    tmp_input_sbp = target_sbp - sbp_offset;
    sbp_cog_offset = tmp_input_sbp - tmpcog;
  }
};

void AutoBalancer::calc_static_balance_point_from_forces(hrp::Vector3& sb_point, const hrp::Vector3& tmpcog, const double ref_com_height, std::vector<hrp::Vector3>& tmp_forces)
{
  hrp::Vector3 denom, nume;
  /* sb_point[m] = nume[kg * m/s^2 * m] / denom[kg * m/s^2] */
  double mass = m_robot->totalMass();
  double mg = mass * gg->get_gravitational_acceleration();
  hrp::Vector3 total_sensor_ref_force = hrp::Vector3::Zero();
  for (size_t i = 0; i < tmp_forces.size(); i++) {
      total_sensor_ref_force += tmp_forces[i];
  }
  hrp::Vector3 total_nosensor_ref_force = mg * hrp::Vector3::UnitZ() - total_sensor_ref_force; // total ref force at the point without sensors, such as torso
  hrp::Vector3 tmp_ext_moment = fix_leg_coords2.pos.cross(total_nosensor_ref_force) + fix_leg_coords2.rot * hrp::Vector3(m_refFootOriginExtMoment.data.x, m_refFootOriginExtMoment.data.y, m_refFootOriginExtMoment.data.z);
  // For MODE_REF_FORCE_RFU_EXT_MOMENT, store previous root position to calculate influence from tmp_ext_moment while walking (basically, root link moves while walking).
  //   Calculate values via fix_leg_coords2 relative/world values.
  static hrp::Vector3 prev_additional_force_applied_pos = fix_leg_coords2.rot.transpose() * (additional_force_applied_link->p-fix_leg_coords2.pos);
  //   If not is_hold_value (not hold value), update prev_additional_force_applied_pos
  if ( !m_refFootOriginExtMomentIsHoldValue.data ) {
      prev_additional_force_applied_pos = fix_leg_coords2.rot.transpose() * (additional_force_applied_link->p-fix_leg_coords2.pos);
  }
  hrp::Vector3 tmp_prev_additional_force_applied_pos = fix_leg_coords2.rot * prev_additional_force_applied_pos + fix_leg_coords2.pos;
  // Calculate SBP
  for (size_t j = 0; j < 2; j++) {
    nume(j) = mg * tmpcog(j);
    denom(j) = mg;
    if ( use_force == MODE_REF_FORCE_RFU_EXT_MOMENT ) {
        //nume(j) += (j==0 ? tmp_ext_moment(1):-tmp_ext_moment(0));
        nume(j) += (tmp_prev_additional_force_applied_pos(j)-additional_force_applied_link->p(j))*total_nosensor_ref_force(2) + (j==0 ? tmp_ext_moment(1):-tmp_ext_moment(0));
        denom(j) -= total_nosensor_ref_force(2);
    } else {
        for ( std::map<std::string, ABCIKparam>::iterator it = ikp.begin(); it != ikp.end(); it++ ) {
            // Check leg_names. leg_names is assumed to be support limb for locomotion, cannot be used for manipulation. If it->first is not included in leg_names, use it for manipulation and static balance point calculation.
            if (std::find(leg_names.begin(), leg_names.end(), it->first) == leg_names.end()) {
                size_t idx = contact_states_index_map[it->first];
                // Force applied point is assumed as end effector
                hrp::Vector3 fpos = it->second.target_link->p + it->second.target_link->R * it->second.localPos;
                nume(j) += ( (fpos(2) - ref_com_height) * tmp_forces[idx](j) - fpos(j) * tmp_forces[idx](2) );
                denom(j) -= tmp_forces[idx](2);
            }
        }
        if ( use_force == MODE_REF_FORCE_WITH_FOOT ) {
            hrp::Vector3 fpos(additional_force_applied_link->p+additional_force_applied_point_offset);
            nume(j) += ( (fpos(2) - ref_com_height) * total_nosensor_ref_force(j) - fpos(j) * total_nosensor_ref_force(2) );
            denom(j) -= total_nosensor_ref_force(2);
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
        hrp::Vector3 rarm_pos = ikp["rarm"].target_p0;
        hrp::Vector3 larm_pos = ikp["larm"].target_p0;
        act_hand_coords.pos = (rarm_pos+larm_pos)/2.0;
        hrp::Vector3 act_y = larm_pos-rarm_pos;
        act_y(2) = 0;
        act_y.normalize();
        hrp::Vector3 ref_y(ref_hand_coords.rot * hrp::Vector3::UnitY());
        ref_y(2) = 0;
        ref_y.normalize();
        dr = hrp::Vector3(0,0,(hrp::Vector3(ref_y.cross(act_y))(2) > 0 ? 1.0 : -1.0) * std::acos(ref_y.dot(act_y))); // fix for rotation
    } else {
      ABCIKparam& tmpikp = ikp[graspless_manip_arm];
      act_hand_coords = rats::coordinates(tmpikp.target_p0,
                                          tmpikp.target_r0);
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

bool AutoBalancer::calc_inital_support_legs(const double& y, std::vector<coordinates>& initial_support_legs_coords, std::vector<leg_type>& initial_support_legs, coordinates& start_ref_coords) {
    switch(gait_type) {
    case BIPED:
        initial_support_legs_coords.assign (1,
                                            y > 0 ?
                                            coordinates(ikp["rleg"].target_p0, ikp["rleg"].target_r0)
                                            : coordinates(ikp["lleg"].target_p0, ikp["lleg"].target_r0));
        initial_support_legs.assign (1, y > 0 ? RLEG : LLEG);
        break;
    case TROT:
        initial_support_legs_coords = (y > 0 ?
                                       boost::assign::list_of(coordinates(ikp["rleg"].target_p0, ikp["rleg"].target_r0))(coordinates(ikp["larm"].target_p0, ikp["larm"].target_r0))
                                       : boost::assign::list_of(coordinates(ikp["lleg"].target_p0, ikp["lleg"].target_r0))(coordinates(ikp["rarm"].target_p0, ikp["rarm"].target_r0))).convert_to_container < std::vector<coordinates> > ();
        initial_support_legs = (y > 0 ? boost::assign::list_of(RLEG)(LARM) : boost::assign::list_of(LLEG)(RARM)).convert_to_container < std::vector<leg_type> > ();
        break;
    case PACE:
        initial_support_legs_coords = (y > 0 ?
                                       boost::assign::list_of(coordinates(ikp["rleg"].target_p0, ikp["rleg"].target_r0))(coordinates(ikp["rarm"].target_p0, ikp["rarm"].target_r0))
                                       : boost::assign::list_of(coordinates(ikp["lleg"].target_p0, ikp["lleg"].target_r0))(coordinates(ikp["larm"].target_p0, ikp["larm"].target_r0))).convert_to_container < std::vector<coordinates> > ();
        initial_support_legs = (y > 0 ? boost::assign::list_of(RLEG)(RARM) : boost::assign::list_of(LLEG)(LARM)).convert_to_container < std::vector<leg_type> > ();
        break;
    case CRAWL:
        std::cerr << "[" << m_profile.instance_name << "] crawl walk[" << gait_type << "] is not implemented yet." << std::endl;
        return false;
    case GALLOP:
        /* at least one leg shoud be in contact */
        std::cerr << "[" << m_profile.instance_name << "] gallop walk[" << gait_type << "] is not implemented yet." << std::endl;
        return false;
    default: break;
    }
    start_ref_coords.pos = (ikp["rleg"].target_p0+ikp["lleg"].target_p0)*0.5;
    mid_rot(start_ref_coords.rot, 0.5, ikp["rleg"].target_r0, ikp["lleg"].target_r0);
    return true;
};

// TODO : Use same code as ZMPDistributor.h
// Solve A * x = b => x = W A^T (A W A^T)-1 b
// => x = W^{1/2} Pinv(A W^{1/2}) b
// Copied from ZMPDistributor.h
void calcWeightedLinearEquation(hrp::dvector& ret, const hrp::dmatrix& A, const hrp::dmatrix& W, const hrp::dvector& b)
{
    hrp::dmatrix W2 = hrp::dmatrix::Zero(W.rows(), W.cols());
    for (size_t i = 0; i < W.rows(); i++) W2(i,i) = std::sqrt(W(i,i));
    hrp::dmatrix Aw = A*W2;
    hrp::dmatrix Aw_inv = hrp::dmatrix::Zero(A.cols(), A.rows());
    hrp::calcPseudoInverse(Aw, Aw_inv);
    ret = W2 * Aw_inv * b;
    //ret = W2 * Aw.colPivHouseholderQr().solve(b);
};

void AutoBalancer::distributeReferenceZMPToWrenches (const hrp::Vector3& _ref_zmp)
{
    // apply inverse system
    // TODO : fix 0.055 (zmp delay)
    hrp::Vector3 tmp_ref_zmp = _ref_zmp + 0.055 * (_ref_zmp - prev_ref_zmp) / m_dt;

    std::vector<hrp::Vector3> cop_pos;
    std::vector<double> limb_gains;
    for (size_t i = 0 ; i < leg_names.size(); i++) {
        ABCIKparam& tmpikp = ikp[leg_names[i]];
        cop_pos.push_back(tmpikp.target_p0 + tmpikp.target_r0 * tmpikp.localR * default_zmp_offsets[i]);
        limb_gains.push_back(m_contactStates.data[contact_states_index_map[leg_names[i]]] ? 1.0 : 0.0);
    }
    size_t ee_num = leg_names.size();
    size_t state_dim = 6*ee_num;
    size_t total_wrench_dim = 5;
    // size_t total_fz = m_robot->totalMass() * gg->get_gravitational_acceleration();
    size_t total_fz = m_ref_force[0].data[2]+m_ref_force[1].data[2];
    //size_t total_wrench_dim = 3;
    hrp::dmatrix Wmat = hrp::dmatrix::Identity(state_dim/2, state_dim/2);
    hrp::dmatrix Gmat = hrp::dmatrix::Zero(total_wrench_dim, state_dim/2);
    // Set Gmat
    //   Fill Fz
    for (size_t j = 0; j < ee_num; j++) {
        if (total_wrench_dim == 3) {
            Gmat(0,3*j+2) = 1.0;
        } else {
            for (size_t k = 0; k < 3; k++) Gmat(k,3*j+k) = 1.0;
        }
    }
    //   Fill Nx and Ny
    for (size_t i = 0; i < total_wrench_dim; i++) {
        for (size_t j = 0; j < ee_num; j++) {
            if ( i == total_wrench_dim-2 ) { // Nx
                Gmat(i,3*j+1) = -(cop_pos[j](2) - tmp_ref_zmp(2));
                Gmat(i,3*j+2) = (cop_pos[j](1) - tmp_ref_zmp(1));
            } else if ( i == total_wrench_dim-1 ) { // Ny
                Gmat(i,3*j) = (cop_pos[j](2) - tmp_ref_zmp(2));
                Gmat(i,3*j+2) = -(cop_pos[j](0) - tmp_ref_zmp(0));
            }
        }
    }
    // Set Wmat
    for (size_t j = 0; j < ee_num; j++) {
        for (size_t i = 0; i < 3; i++) {
            if (ee_num == 2)
                Wmat(i+j*3, i+j*3) = Wmat(i+j*3, i+j*3) * limb_gains[j] * (i==2? 1.0 : 0.01);
            else
                Wmat(i+j*3, i+j*3) = Wmat(i+j*3, i+j*3) * limb_gains[j];
        }
    }
    // Ret is wrench around cop_pos
    //   f_cop = f_ee
    //   n_ee = (cop_pos - ee_pos) x f_cop + n_cop
    hrp::dvector ret(state_dim/2);
    hrp::dvector total_wrench = hrp::dvector::Zero(total_wrench_dim);
    total_wrench(total_wrench_dim-5) = m_ref_force[0].data[0]+m_ref_force[1].data[0];
    total_wrench(total_wrench_dim-4) = m_ref_force[0].data[1]+m_ref_force[1].data[1];
    total_wrench(total_wrench_dim-3) = total_fz;
    calcWeightedLinearEquation(ret, Gmat, Wmat, total_wrench);
    if (DEBUGP) {
        std::cerr << "[" << m_profile.instance_name << "] distributeReferenceZMPToWrenches" << std::endl;
    }
    for (size_t i = 0 ; i < leg_names.size(); i++) {
        size_t fidx = contact_states_index_map[leg_names[i]];
        ABCIKparam& tmpikp = ikp[leg_names[i]];
        hrp::Vector3 f_ee(ret(3*i), ret(3*i+1), ret(3*i+2));
        //hrp::Vector3 tmp_ee_pos = tmpikp.target_p0 + tmpikp.target_r0 * tmpikp.localPos;
        hrp::Vector3 tmp_ee_pos = tmpikp.target_p0;
        hrp::Vector3 n_ee = (cop_pos[i]-tmp_ee_pos).cross(f_ee); // n_cop = 0
        m_force[fidx].data[0] = f_ee(0);
        m_force[fidx].data[1] = f_ee(1);
        m_force[fidx].data[2] = f_ee(2);
        m_force[fidx].data[3] = n_ee(0);
        m_force[fidx].data[4] = n_ee(1);
        m_force[fidx].data[5] = n_ee(2);
        if (DEBUGP) {
            std::cerr << "[" << m_profile.instance_name << "]   "
                      << "ref_force  [" << leg_names[i] << "] " << f_ee.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[N], "
                      << "ref_moment [" << leg_names[i] << "] " << n_ee.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[Nm]" << std::endl;
        }
    }
    if (DEBUGP) {
        std::cerr << "[" << m_profile.instance_name << "]   Gmat = " << Gmat.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[N,Nm]" << std::endl;
        std::cerr << "[" << m_profile.instance_name << "]   total_wrench = " << total_wrench.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[N,Nm]" << std::endl;
        hrp::dvector tmp(total_wrench.size());
        tmp = Gmat*ret;
        std::cerr << "[" << m_profile.instance_name << "]   Gmat*ret = " << tmp.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[N,Nm]" << std::endl;
        std::cerr << "[" << m_profile.instance_name << "]   (Gmat*ret-total_wrench) = " << (tmp-total_wrench).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[N,Nm]" << std::endl;
        std::cerr << "[" << m_profile.instance_name << "]   ret = " << ret.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[N,Nm]" << std::endl;
        std::cerr << "[" << m_profile.instance_name << "]   Wmat(diag) = [";
        for (size_t j = 0; j < ee_num; j++) {
            for (size_t i = 0; i < 3; i++) {
                std::cerr << Wmat(i+j*3, i+j*3) << " ";
            }
        }
        std::cerr << "]" << std::endl;
    }
};

std::string AutoBalancer::getUseForceModeString ()
{
    switch (use_force) {
    case OpenHRP::AutoBalancerService::MODE_NO_FORCE:
        return "MODE_NO_FORCE";
    case OpenHRP::AutoBalancerService::MODE_REF_FORCE:
        return "MODE_REF_FORCE";
    case OpenHRP::AutoBalancerService::MODE_REF_FORCE_WITH_FOOT:
        return "MODE_REF_FORCE_WITH_FOOT";
    case OpenHRP::AutoBalancerService::MODE_REF_FORCE_RFU_EXT_MOMENT:
        return "MODE_REF_FORCE_RFU_EXT_MOMENT";
    default:
        return "";
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


