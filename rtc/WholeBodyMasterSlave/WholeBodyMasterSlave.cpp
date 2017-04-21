// -*- C++ -*-
/*!
 * @file  WholeBodyMasterSlave.cpp
 * @brief WholeBodyMasterSlave component
 * $Date$
 *
 * $Id$
 */

#include <rtm/CorbaNaming.h>
#include <hrpModel/Link.h>
#include <hrpModel/Sensor.h>
#include <hrpModel/ModelLoaderUtil.h>
#include "WholeBodyMasterSlave.h"
#include <hrpModel/JointPath.h>
#include <hrpUtil/MatrixSolvers.h>
#include "hrpsys/util/Hrpsys.h"


typedef coil::Guard<coil::Mutex> Guard;
using namespace rats;

// Module specification
// <rtc-template block="module_spec">
static const char* WholeBodyMasterSlave_spec[] =
    {
        "implementation_id", "WholeBodyMasterSlave",
        "type_name",         "WholeBodyMasterSlave",
        "description",       "wholebodymasterslave component",
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

WholeBodyMasterSlave::WholeBodyMasterSlave(RTC::Manager* manager)
    : RTC::DataFlowComponentBase(manager),
      // <rtc-template block="initializer">
      m_qRefIn("qRef", m_qRef),
      m_zmpIn("zmpIn", m_zmp),
      m_basePosIn("basePosIn", m_basePos),
      m_baseRpyIn("baseRpyIn", m_baseRpy),
      m_optionalDataIn("optionalData", m_optionalData),
      m_emergencySignalIn("emergencySignal", m_emergencySignal),
      //ishiguro
      m_htzmpIn("htzmpIn", m_htzmp),
      m_htrfwIn("htrfwIn", m_htrfw),
      m_htlfwIn("htlfwIn", m_htlfw),
      m_htcomIn("htcomIn", m_htcom),
      m_htrfIn("htrfIn", m_htrf),
      m_htlfIn("htlfIn", m_htlf),
      m_htrhIn("htrhIn", m_htrh),
      m_htlhIn("htlhIn", m_htlh),
      m_actzmpIn("actzmpIn", m_actzmp),
      m_htheadIn("htheadIn", m_hthead),

      m_htcom_dbgOut("htcom_dbgOut", m_htcom_dbg),
      m_htrf_dbgOut("htrf_dbgOut", m_htrf_dbg),
      m_htlf_dbgOut("htlf_dbgOut", m_htlf_dbg),
      m_htrh_dbgOut("htrh_dbgOut", m_htrh_dbg),
      m_htlh_dbgOut("htlh_dbgOut", m_htlh_dbg),
      m_hthead_dbgOut("hthead_dbgOut", m_hthead_dbg),
      m_htzmp_dbgOut("htzmp_dbgOut", m_htzmp_dbg),
      m_htrfw_dbgOut("htrfw_dbgOut", m_htrfw_dbg),
      m_htlfw_dbgOut("htlfw_dbgOut", m_htlfw_dbg),
      m_rpcom_dbgOut("rpcom_dbgOut", m_rpcom_dbg),
      m_rprf_dbgOut("rprf_dbgOut", m_rprf_dbg),
      m_rplf_dbgOut("rplf_dbgOut", m_rplf_dbg),
      m_rprh_dbgOut("rprh_dbgOut", m_rprh_dbg),
      m_rplh_dbgOut("rplh_dbgOut", m_rplh_dbg),
      m_rphead_dbgOut("rphead_dbgOut", m_rphead_dbg),
      m_rpzmp_dbgOut("rpzmp_dbgOut", m_rpzmp_dbg),
      m_rpdcp_dbgOut("rpdcp_dbgOut", m_rpdcp_dbg),
      m_rpacp_dbgOut("rpacp_dbgOut", m_rpacp_dbg),

      m_invdyn_dbgOut("invdyn_dbgOut", m_invdyn_dbg),

      m_qOut("q", m_qRef),
      m_zmpOut("zmpOut", m_zmp),
      m_basePosOut("basePosOut", m_basePos),
      m_baseRpyOut("baseRpyOut", m_baseRpy),
      m_optionalDataOut("optionalDataOut", m_optionalData2),
      m_baseTformOut("baseTformOut", m_baseTform),
      m_basePoseOut("basePoseOut", m_basePose),
      m_accRefOut("accRef", m_accRef),
      m_contactStatesOut("contactStates", m_contactStates),
      m_toeheelRatioOut("toeheelRatio", m_toeheelRatio),
      m_controlSwingSupportTimeOut("controlSwingSupportTime", m_controlSwingSupportTime),
      m_walkingStatesOut("walkingStates", m_walkingStates),
      m_sbpCogOffsetOut("sbpCogOffset", m_sbpCogOffset),
      m_cogOut("cogOut", m_cog),
      m_WholeBodyMasterSlaveServicePort("WholeBodyMasterSlaveService"),
      // </rtc-template>
//      gait_type(BIPED),
      m_robot(hrp::BodyPtr()),
      m_debugLevel(0)
{
    m_service0.wholebodymasterslave(this);
}

WholeBodyMasterSlave::~WholeBodyMasterSlave()
{
}


RTC::ReturnCode_t WholeBodyMasterSlave::onInitialize()
{
    std::cerr << "[" << m_profile.instance_name << "] onInitialize()" << std::endl;
    bindParameter("debugLevel", m_debugLevel, "0");

    // Registration: InPort/OutPort/Service
    // <rtc-template block="registration">
    // Set InPort buffers
    addInPort("qRef", m_qRefIn);
    addInPort("zmpIn", m_zmpIn);
    addInPort("basePosIn", m_basePosIn);
    addInPort("baseRpyIn", m_baseRpyIn);
    addInPort("optionalData", m_optionalDataIn);
    addInPort("emergencySignal", m_emergencySignalIn);
    //ishiguro
    addInPort("htzmpIn", m_htzmpIn);
    addInPort("htrfwIn", m_htrfwIn);
    addInPort("htlfwIn", m_htlfwIn);
    addInPort("htcomIn", m_htcomIn);
    addInPort("htrfIn", m_htrfIn);
    addInPort("htlfIn", m_htlfIn);
    addInPort("htrhIn", m_htrhIn);
    addInPort("htlhIn", m_htlhIn);
    addInPort("actzmpIn", m_actzmpIn);
    addInPort("htheadIn", m_htheadIn);

    addOutPort("htcom_dbgOut", m_htcom_dbgOut);
    addOutPort("htrf_dbgOut", m_htrf_dbgOut);
    addOutPort("htlf_dbgOut", m_htlf_dbgOut);
    addOutPort("htrh_dbgOut", m_htrh_dbgOut);
    addOutPort("htlh_dbgOut", m_htlh_dbgOut);
    addOutPort("hthead_dbgOut", m_hthead_dbgOut);
    addOutPort("htzmp_dbgOut", m_htzmp_dbgOut);
    addOutPort("htrfw_dbgOut", m_htrfw_dbgOut);
    addOutPort("htlfw_dbgOut", m_htlfw_dbgOut);
    addOutPort("rpcom_dbgOut", m_rpcom_dbgOut);
    addOutPort("rprf_dbgOut", m_rprf_dbgOut);
    addOutPort("rplf_dbgOut", m_rplf_dbgOut);
    addOutPort("rprh_dbgOut", m_rprh_dbgOut);
    addOutPort("rplh_dbgOut", m_rplh_dbgOut);
    addOutPort("rphead_dbgOut", m_rphead_dbgOut);
    addOutPort("rpzmp_dbgOut", m_rpzmp_dbgOut);
    addOutPort("rpdcp_dbgOut", m_rpdcp_dbgOut);
    addOutPort("rpacp_dbgOut", m_rpacp_dbgOut);

    addOutPort("invdyn_dbgOut", m_invdyn_dbgOut);

    // Set OutPort buffer
    addOutPort("q", m_qOut);
    addOutPort("zmpOut", m_zmpOut);
    addOutPort("basePosOut", m_basePosOut);
    addOutPort("baseRpyOut", m_baseRpyOut);
    addOutPort("optionalDataOut", m_optionalDataOut);
//    addOutPort("baseTformOut", m_baseTformOut);
//    addOutPort("basePoseOut", m_basePoseOut);
//    addOutPort("accRef", m_accRefOut);
//    addOutPort("contactStates", m_contactStatesOut);
//    addOutPort("toeheelRatio", m_toeheelRatioOut);
//    addOutPort("controlSwingSupportTime", m_controlSwingSupportTimeOut);
//    addOutPort("cogOut", m_cogOut);
//    addOutPort("walkingStates", m_walkingStatesOut);
//    addOutPort("sbpCogOffset", m_sbpCogOffsetOut);
  
    // Set service provider to Ports
    m_WholeBodyMasterSlaveServicePort.registerProvider("service0", "WholeBodyMasterSlaveService", m_service0);
  
    // Set service consumers to Ports
  
    // Set CORBA Service Ports
    addPort(m_WholeBodyMasterSlaveServicePort);
  
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
    //ishiguro
    m_htrfw.data.length(6);
    m_htlfw.data.length(6);

    m_optionalData2.data.length(4*2);//これいいのか？
    for(int i=0;i<4*2;i++)m_optionalData2.data[i] = 0;

//    control_mode = MODE_IDLE;
    loop = 0;
//
//    // setting from conf file
//    // GaitGenerator requires abc_leg_offset and abc_stride_parameter in robot conf file
//    // setting leg_pos from conf file
//    coil::vstring leg_offset_str = coil::split(prop["abc_leg_offset"], ",");
//    leg_names.push_back("rleg");
//    leg_names.push_back("lleg");
//
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
//        ikp.insert(std::pair<std::string, ABCIKparam>(ee_name , tp));
//        ee_vec.push_back(ee_name);
        std::cerr << "[" << m_profile.instance_name << "] End Effector [" << ee_name << "]" << std::endl;
        std::cerr << "[" << m_profile.instance_name << "]   target = " << fik->ikp[ee_name].target_link->name << ", base = " << ee_base << std::endl;
        std::cerr << "[" << m_profile.instance_name << "]   offset_pos = " << tp.localPos.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "    [", "]")) << "[m]" << std::endl;
        std::cerr << "[" << m_profile.instance_name << "]   has_toe_joint = " << (tp.has_toe_joint?"true":"false") << std::endl;
        contact_states_index_map.insert(std::pair<std::string, size_t>(ee_name, i));
      }
//      m_contactStates.data.length(num);
//      m_toeheelRatio.data.length(num);
//      if (ikp.find("rleg") != ikp.end() && ikp.find("lleg") != ikp.end()) {
//        m_contactStates.data[contact_states_index_map["rleg"]] = true;
//        m_contactStates.data[contact_states_index_map["lleg"]] = true;
//      }
//      if (ikp.find("rarm") != ikp.end() && ikp.find("larm") != ikp.end()) {
//        m_contactStates.data[contact_states_index_map["rarm"]] = false;
//        m_contactStates.data[contact_states_index_map["larm"]] = false;
//      }
//      m_controlSwingSupportTime.data.length(num);
//      for (size_t i = 0; i < num; i++) m_controlSwingSupportTime.data[i] = 0.0;
//      for (size_t i = 0; i < num; i++) m_toeheelRatio.data[i] = rats::no_using_toe_heel_ratio;
    }
//    std::vector<hrp::Vector3> leg_pos;
//    if (leg_offset_str.size() > 0) {
//      hrp::Vector3 leg_offset;
//      for (size_t i = 0; i < 3; i++) coil::stringTo(leg_offset(i), leg_offset_str[i].c_str());
//      std::cerr << "[" << m_profile.instance_name << "] abc_leg_offset = " << leg_offset.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "    [", "]")) << "[m]" << std::endl;
//      leg_pos.push_back(hrp::Vector3(-1*leg_offset));
//      leg_pos.push_back(hrp::Vector3(leg_offset));
//    }
//    if (leg_pos.size() < ikp.size()) {
//        size_t tmp_leg_pos_size = leg_pos.size();
//        for (size_t i = 0; i < ikp.size() - tmp_leg_pos_size; i++) {
//            leg_pos.push_back(hrp::Vector3::Zero());
//        }
//    }
//
//    std::vector<std::pair<hrp::Link*, hrp::Link*> > interlocking_joints;
//    readInterlockingJointsParamFromProperties(interlocking_joints, m_robot, prop["interlocking_joints"], std::string(m_profile.instance_name));
//    if (interlocking_joints.size() > 0) {
//        fik->initializeInterlockingJoints(interlocking_joints);
//    }
//
//    zmp_offset_interpolator = new interpolator(ikp.size()*3, m_dt);
//    zmp_offset_interpolator->setName(std::string(m_profile.instance_name)+" zmp_offset_interpolator");
//    zmp_transition_time = 1.0;
//    transition_interpolator = new interpolator(1, m_dt, interpolator::HOFFARBIB, 1);
//    transition_interpolator->setName(std::string(m_profile.instance_name)+" transition_interpolator");
//    transition_interpolator_ratio = 0.0;
//    adjust_footstep_interpolator = new interpolator(1, m_dt, interpolator::HOFFARBIB, 1);
//    adjust_footstep_interpolator->setName(std::string(m_profile.instance_name)+" adjust_footstep_interpolator");
//    transition_time = 2.0;
//    adjust_footstep_transition_time = 2.0;
//    leg_names_interpolator = new interpolator(1, m_dt, interpolator::HOFFARBIB, 1);
//    leg_names_interpolator->setName(std::string(m_profile.instance_name)+" leg_names_interpolator");
//    leg_names_interpolator_ratio = 1.0;
//
//    // setting stride limitations from conf file
//    double stride_fwd_x_limit = 0.15;
//    double stride_y_limit = 0.05;
//    double stride_th_limit = 10;
//    double stride_bwd_x_limit = 0.05;
//    std::cerr << "[" << m_profile.instance_name << "] abc_stride_parameter : " << stride_fwd_x_limit << "[m], " << stride_y_limit << "[m], " << stride_th_limit << "[deg], " << stride_bwd_x_limit << "[m]" << std::endl;
//    if (default_zmp_offsets.size() == 0) {
//      for (size_t i = 0; i < ikp.size(); i++) default_zmp_offsets.push_back(hrp::Vector3::Zero());
//    }
//    if (leg_offset_str.size() > 0) {
//      gg = ggPtr(new rats::gait_generator(m_dt, leg_pos, leg_names, stride_fwd_x_limit/*[m]*/, stride_y_limit/*[m]*/, stride_th_limit/*[deg]*/, stride_bwd_x_limit/*[m]*/));
//      gg->set_default_zmp_offsets(default_zmp_offsets);
//    }
//    gg_is_walking = gg_solved = false;
//    m_walkingStates.data = false;
//    fix_leg_coords = coordinates();
//
//    // load virtual force sensors
//    readVirtualForceSensorParamFromProperties(m_vfs, m_robot, prop["virtual_force_sensor"], std::string(m_profile.instance_name));
//    // ref force port
//    unsigned int npforce = m_robot->numSensors(hrp::Sensor::FORCE);
//    unsigned int nvforce = m_vfs.size();
//    unsigned int nforce  = npforce + nvforce;
//    // check number of force sensors
//    if (nforce < m_contactStates.data.length()) {
//        std::cerr << "[" << m_profile.instance_name << "] WARNING! This robot model has less force sensors(" << nforce;
//        std::cerr << ") than end-effector settings(" << m_contactStates.data.length() << ") !" << std::endl;
//    }
//
//    m_ref_force.resize(nforce);
//    m_ref_forceIn.resize(nforce);
//    m_force.resize(nforce);
//    m_ref_forceOut.resize(nforce);
//    m_limbCOPOffset.resize(nforce);
//    m_limbCOPOffsetOut.resize(nforce);
//    for (unsigned int i=0; i<npforce; i++){
//        sensor_names.push_back(m_robot->sensor(hrp::Sensor::FORCE, i)->name);
//    }
//    for (unsigned int i=0; i<nvforce; i++){
//        for ( std::map<std::string, hrp::VirtualForceSensorParam>::iterator it = m_vfs.begin(); it != m_vfs.end(); it++ ) {
//            if (it->second.id == (int)i) sensor_names.push_back(it->first);
//        }
//    }
//    // set ref force port
//    std::cerr << "[" << m_profile.instance_name << "] force sensor ports (" << nforce << ")" << std::endl;
//    for (unsigned int i=0; i<nforce; i++){
//        m_ref_forceIn[i] = new InPort<TimedDoubleSeq>(std::string("ref_"+sensor_names[i]).c_str(), m_ref_force[i]);
//        m_ref_force[i].data.length(6);
//        registerInPort(std::string("ref_"+sensor_names[i]).c_str(), *m_ref_forceIn[i]);
//        std::cerr << "[" << m_profile.instance_name << "]   name = " << std::string("ref_"+sensor_names[i]) << std::endl;
//        ref_forces.push_back(hrp::Vector3(0,0,0));
//    }
//    // set force port
//    for (unsigned int i=0; i<nforce; i++){
//        m_ref_forceOut[i] = new OutPort<TimedDoubleSeq>(std::string(sensor_names[i]).c_str(), m_force[i]);
//        m_force[i].data.length(6);
//        m_force[i].data[0] = m_force[i].data[1] = m_force[i].data[2] = 0.0;
//        m_force[i].data[3] = m_force[i].data[4] = m_force[i].data[5] = 0.0;
//        registerOutPort(std::string(sensor_names[i]).c_str(), *m_ref_forceOut[i]);
//        std::cerr << "[" << m_profile.instance_name << "]   name = " << std::string(sensor_names[i]) << std::endl;
//    }
//    // set limb cop offset port
//    std::cerr << "[" << m_profile.instance_name << "] limbCOPOffset ports (" << nforce << ")" << std::endl;
//    for (unsigned int i=0; i<nforce; i++){
//        std::string nm("limbCOPOffset_"+sensor_names[i]);
//        m_limbCOPOffsetOut[i] = new OutPort<TimedPoint3D>(nm.c_str(), m_limbCOPOffset[i]);
//        registerOutPort(nm.c_str(), *m_limbCOPOffsetOut[i]);
//        m_limbCOPOffset[i].data.x = m_limbCOPOffset[i].data.y = m_limbCOPOffset[i].data.z = 0.0;
//        std::cerr << "[" << m_profile.instance_name << "]   name = " << nm << std::endl;
//    }
//    sbp_offset = hrp::Vector3(0,0,0);
//    sbp_cog_offset = hrp::Vector3(0,0,0);
//    //use_force = MODE_NO_FORCE;
//    use_force = MODE_REF_FORCE;
//
    if (fik->ikp.find("rleg") != fik->ikp.end() && fik->ikp.find("lleg") != fik->ikp.end()) {
      is_legged_robot = true;
    } else {
      is_legged_robot = false;
    }
//
//    m_accRef.data.ax = m_accRef.data.ay = m_accRef.data.az = 0.0;
//    prev_imu_sensor_vel = hrp::Vector3::Zero();
//
//    graspless_manip_mode = false;
//    graspless_manip_arm = "arms";
//    graspless_manip_p_gain = hrp::Vector3::Zero();
//
//    is_stop_mode = false;
//    is_hand_fix_mode = false;

    //for HumanSynchronizer
    hsp = boost::shared_ptr<HumanSynchronizer>(new HumanSynchronizer());
//    if(leg_pos.size()>=2){ hsp->init_wld_rp_rfeepos = leg_pos[0]; hsp->init_wld_rp_lfeepos = leg_pos[1]; }

//    hrp::Sensor* sen = m_robot->sensor<hrp::RateGyroSensor>("gyrometer");
//    if (sen == NULL) {
//        std::cerr << "[" << m_profile.instance_name << "] WARNING! This robot model has no GyroSensor named 'gyrometer'! " << std::endl;
//    }
    return RTC::RTC_OK;
}


RTC::ReturnCode_t WholeBodyMasterSlave::onFinalize()
{
//  delete zmp_offset_interpolator;
//  delete transition_interpolator;
//  delete adjust_footstep_interpolator;
//  delete leg_names_interpolator;
  return RTC::RTC_OK;
}

RTC::ReturnCode_t WholeBodyMasterSlave::onActivated(RTC::UniqueId ec_id)
{
    std::cerr << "[" << m_profile.instance_name<< "] onActivated(" << ec_id << ")" << std::endl;
    
    return RTC::RTC_OK;
}

RTC::ReturnCode_t WholeBodyMasterSlave::onDeactivated(RTC::UniqueId ec_id)
{
  std::cerr << "[" << m_profile.instance_name<< "] onDeactivated(" << ec_id << ")" << std::endl;
//  Guard guard(m_mutex);
//  if (control_mode == MODE_ABC) {
//    control_mode = MODE_SYNC_TO_IDLE;
//    double tmp_ratio = 0.0;
//    transition_interpolator->setGoal(&tmp_ratio, m_dt, true); // sync in one controller loop
//  }
  return RTC::RTC_OK;
}




#define DEBUGP ((m_debugLevel==1 && loop%200==0) || m_debugLevel > 1 )
//#define DEBUGP2 ((loop%200==0))
#define DEBUGP2 (false)
RTC::ReturnCode_t WholeBodyMasterSlave::onExecute(RTC::UniqueId ec_id)
{
  // std::cerr << "WholeBodyMasterSlave::onExecute(" << ec_id << ")" << std::endl;
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
//      input_zmp(0) = m_zmp.data.x;
//      input_zmp(1) = m_zmp.data.y;
//      input_zmp(2) = m_zmp.data.z;
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
    }
    //for HumanSynchronizer
    if (m_htzmpIn.isNew()){	m_htzmpIn.read(); }
    if (m_htrfwIn.isNew()){ m_htrfwIn.read(); HumanSynchronizer::DoubleSeqToWrench6(m_htrfw.data,hsp->hp_wld_raw.getw("rfw")); }
    if (m_htlfwIn.isNew()){ m_htlfwIn.read(); HumanSynchronizer::DoubleSeqToWrench6(m_htlfw.data,hsp->hp_wld_raw.getw("lfw")); }
    if (m_htcomIn.isNew()){ m_htcomIn.read(); HumanSynchronizer::Pose3DToHRPPose3D(m_htcom.data,hsp->hp_wld_raw.getP("com")); }
    if (m_htrfIn.isNew()) { m_htrfIn.read();  HumanSynchronizer::Pose3DToHRPPose3D(m_htrf.data,hsp->hp_wld_raw.getP("rf")); }
    if (m_htlfIn.isNew()) { m_htlfIn.read();  HumanSynchronizer::Pose3DToHRPPose3D(m_htlf.data,hsp->hp_wld_raw.getP("lf")); }
    if (m_htrhIn.isNew()) { m_htrhIn.read();  HumanSynchronizer::Pose3DToHRPPose3D(m_htrh.data,hsp->hp_wld_raw.getP("rh"));}
    if (m_htlhIn.isNew()) { m_htlhIn.read();  HumanSynchronizer::Pose3DToHRPPose3D(m_htlh.data,hsp->hp_wld_raw.getP("lh"));}
    if (m_htheadIn.isNew()){ m_htheadIn.read(); HumanSynchronizer::Pose3DToHRPPose3D(m_hthead.data,hsp->head_cam_pose);}
    if (m_actzmpIn.isNew()){m_actzmpIn.read(); }

//    if(ikp.count("rarm"))ikp["rarm"].is_active = false;
//    if(ikp.count("larm"))ikp["larm"].is_active = false;
//    m_contactStates.data[contact_states_index_map["rleg"]] = hsp->is_rf_contact;
//    m_contactStates.data[contact_states_index_map["lleg"]] = hsp->is_lf_contact;
//
//
//    // Calculation
//    Guard guard(m_mutex);
//    hrp::Vector3 ref_basePos;
//    hrp::Matrix33 ref_baseRot;
//    hrp::Vector3 rel_ref_zmp; // ref zmp in base frame
    if ( is_legged_robot ) {
//      // For parameters
//      fik->storeCurrentParameters();
//      getTargetParameters();
//      // Get transition ratio
//      bool is_transition_interpolator_empty = transition_interpolator->isEmpty();
//      if (!is_transition_interpolator_empty) {
//        transition_interpolator->get(&transition_interpolator_ratio, true);
//      } else {
//        transition_interpolator_ratio = (control_mode == MODE_IDLE) ? 0.0 : 1.0;
//      }
        hsp->baselinkpose.p = m_robot->rootLink()->p;
        hsp->baselinkpose.rpy = hrp::rpyFromRot(m_robot->rootLink()->R);
        if(!hsp->isHumanSyncOn()){
          m_robot->rootLink()->p = input_basePos;
          m_robot->rootLink()->R = input_baseRot;
          for ( unsigned int i = 0; i < m_robot->numJoints(); i++ ){
            m_robot->joint(i)->q = m_qRef.data[i];
          }

//          cout<<"m_robot->calcCM()"<<m_robot->calcCM().transpose()<<endl;
//          cout<<"m_robot->p"<<m_robot->rootLink()->p.transpose()<<endl;
//          cout<<"m_robot->R"<<m_robot->rootLink()->R<<endl;

          m_robot->calcForwardKinematics();
//          solveFullbodyIK();
        }

        processWholeBodyMasterSlave();

        /////// Inverse Dynamics /////////
//        if(!idsb.is_initialized){
//          idsb.setInitState(m_robot, m_dt);
//          invdyn_zmp_filters.resize(3);
//          for(int i=0;i<3;i++){
//            invdyn_zmp_filters[i].setParameterAsBiquad(25, 1/std::sqrt(2), 1.0/m_dt);
//            invdyn_zmp_filters[i].reset(ref_zmp(i));
//          }
//          idsb2.setInitState(m_robot, m_dt);
//          invdyn_zmp_filters2.resize(3);
//          for(int i=0;i<3;i++){
//            invdyn_zmp_filters2[i].setParameterAsBiquad(25, 1/std::sqrt(2), 1.0/m_dt);
//            invdyn_zmp_filters2[i].reset(ref_zmp(i));
//          }
//        }
//        calcAccelerationsForInverseDynamics(m_robot, idsb);
//        hrp::Vector3 ref_zmp_invdyn;
//        calcWorldZMPFromInverseDynamics(m_robot, idsb, ref_zmp_invdyn);
//        for(int i=0;i<3;i++) ref_zmp_invdyn(i) = invdyn_zmp_filters[i].passFilter(ref_zmp_invdyn(i));
//        updateInvDynStateBuffer(idsb);


//        calcDynamicsFilterCompensation(ref_zmp, ref_zmp_invdyn);


//        calcAccelerationsForInverseDynamics(m_robot, idsb2);
//        calcWorldZMPFromInverseDynamics(m_robot, idsb2, ref_zmp_invdyn2);
//        for(int i=0;i<3;i++) ref_zmp_invdyn2(i) = invdyn_zmp_filters2[i].passFilter(ref_zmp_invdyn2(i));
//        updateInvDynStateBuffer(idsb2);


//        rel_ref_zmp = m_robot->rootLink()->R.transpose() * (ref_zmp - m_robot->rootLink()->p);
//      // Transition
//      if (!is_transition_interpolator_empty) {
//        // transition_interpolator_ratio 0=>1 : IDLE => ABC
//        // transition_interpolator_ratio 1=>0 : ABC => IDLE
//        ref_basePos = (1-transition_interpolator_ratio) * input_basePos + transition_interpolator_ratio * m_robot->rootLink()->p;
//        rel_ref_zmp = (1-transition_interpolator_ratio) * input_zmp + transition_interpolator_ratio * rel_ref_zmp;
//        rats::mid_rot(ref_baseRot, transition_interpolator_ratio, input_baseRot, m_robot->rootLink()->R);
//        for ( unsigned int i = 0; i < m_robot->numJoints(); i++ ) {
//          m_robot->joint(i)->q = (1-transition_interpolator_ratio) * m_qRef.data[i] + transition_interpolator_ratio * m_robot->joint(i)->q;
//        }
//        for (unsigned int i=0; i< m_force.size(); i++) {
//            for (unsigned int j=0; j<6; j++) {
//                m_force[i].data[j] = transition_interpolator_ratio * m_force[i].data[j] + (1-transition_interpolator_ratio) * m_ref_force[i].data[j];
//            }
//        }
//        for (unsigned int i=0; i< m_limbCOPOffset.size(); i++) {
//            // transition (TODO:set stopABCmode value instead of 0)
//            m_limbCOPOffset[i].data.x = transition_interpolator_ratio * m_limbCOPOffset[i].data.x;// + (1-transition_interpolator_ratio) * 0;
//            m_limbCOPOffset[i].data.y = transition_interpolator_ratio * m_limbCOPOffset[i].data.y;// + (1-transition_interpolator_ratio) * 0;
//            m_limbCOPOffset[i].data.z = transition_interpolator_ratio * m_limbCOPOffset[i].data.z;// + (1-transition_interpolator_ratio) * 0;
//        }
//      } else {
//        ref_basePos = m_robot->rootLink()->p;
//        ref_baseRot = m_robot->rootLink()->R;
//      }
//      // mode change for sync
//      if (control_mode == MODE_SYNC_TO_ABC) {
//        control_mode = MODE_ABC;
//      } else if (control_mode == MODE_SYNC_TO_IDLE && transition_interpolator->isEmpty() ) {
//        std::cerr << "[" << m_profile.instance_name << "] [" << m_qRef.tm
//                  << "] Finished cleanup" << std::endl;
//        control_mode = MODE_IDLE;
//      }
    }
//
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
      m_basePos.data.x = m_robot->rootLink()->p(0);
      m_basePos.data.y = m_robot->rootLink()->p(1);
      m_basePos.data.z = m_robot->rootLink()->p(2);
      m_basePos.tm = m_qRef.tm;
      // baseRpy
      hrp::Vector3 baseRpy = hrp::rpyFromRot(m_robot->rootLink()->R);
      m_baseRpy.data.r = baseRpy(0);
      m_baseRpy.data.p = baseRpy(1);
      m_baseRpy.data.y = baseRpy(2);
      m_baseRpy.tm = m_qRef.tm;
//      // baseTform
//      double *tform_arr = m_baseTform.data.get_buffer();
//      tform_arr[0] = m_basePos.data.x;
//      tform_arr[1] = m_basePos.data.y;
//      tform_arr[2] = m_basePos.data.z;
//      hrp::setMatrix33ToRowMajorArray(ref_baseRot, tform_arr, 3);
//      m_baseTform.tm = m_qRef.tm;
//      // basePose
//      m_basePose.data.position.x = m_basePos.data.x;
//      m_basePose.data.position.y = m_basePos.data.y;
//      m_basePose.data.position.z = m_basePos.data.z;
//      m_basePose.data.orientation.r = m_baseRpy.data.r;
//      m_basePose.data.orientation.p = m_baseRpy.data.p;
//      m_basePose.data.orientation.y = m_baseRpy.data.y;
//      m_basePose.tm = m_qRef.tm;
      // zmp
      m_zmp.data.x = rel_ref_zmp(0);
      m_zmp.data.y = rel_ref_zmp(1);
      m_zmp.data.z = rel_ref_zmp(2);
      m_zmp.tm = m_qRef.tm;
//      // cog
//      m_cog.data.x = ref_cog(0);
//      m_cog.data.y = ref_cog(1);
//      m_cog.data.z = ref_cog(2);
//      m_cog.tm = m_qRef.tm;
//      // sbpCogOffset
//      m_sbpCogOffset.data.x = sbp_cog_offset(0);
//      m_sbpCogOffset.data.y = sbp_cog_offset(1);
//      m_sbpCogOffset.data.z = sbp_cog_offset(2);
//      m_sbpCogOffset.tm = m_qRef.tm;
//      // write
      m_basePosOut.write();
      m_baseRpyOut.write();
//      m_baseTformOut.write();
//      m_basePoseOut.write();
      m_zmpOut.write();
      m_optionalDataOut.write();
//      m_cogOut.write();
//      m_sbpCogOffsetOut.write();
//
//      // reference acceleration
//      hrp::Sensor* sen = m_robot->sensor<hrp::RateGyroSensor>("gyrometer");
//      if (sen != NULL) {
//          hrp::Vector3 imu_sensor_pos = sen->link->p + sen->link->R * sen->localPos;
//          hrp::Vector3 imu_sensor_vel = (imu_sensor_pos - prev_imu_sensor_pos)/m_dt;
//          // convert to imu sensor local acceleration
//          hrp::Vector3 acc = (sen->link->R * sen->localR).transpose() * (imu_sensor_vel - prev_imu_sensor_vel)/m_dt;
//          m_accRef.data.ax = acc(0); m_accRef.data.ay = acc(1); m_accRef.data.az = acc(2);
//          m_accRefOut.write();
//          prev_imu_sensor_pos = imu_sensor_pos;
//          prev_imu_sensor_vel = imu_sensor_vel;
//      }
//
//      // control parameters
//      m_contactStates.tm = m_qRef.tm;
//      m_contactStatesOut.write();
//      m_controlSwingSupportTime.tm = m_qRef.tm;
//      m_controlSwingSupportTimeOut.write();
//      m_toeheelRatio.tm = m_qRef.tm;
//      m_toeheelRatioOut.write();
//      m_walkingStates.data = gg_is_walking;
//      m_walkingStates.tm = m_qRef.tm;
//      m_walkingStatesOut.write();
//
//      for (unsigned int i=0; i<m_ref_forceOut.size(); i++){
//          m_force[i].tm = m_qRef.tm;
//          m_ref_forceOut[i]->write();
//      }
//
//      for (unsigned int i=0; i<m_limbCOPOffsetOut.size(); i++){
//          m_limbCOPOffset[i].tm = m_qRef.tm;
//          m_limbCOPOffsetOut[i]->write();
//      }
    }

    //ishiguro dbg plot
    m_htcom.tm = m_qRef.tm;
    HumanSynchronizer::HRPPose3DToPose3D(hsp->hp_plot.getP("com"),m_htcom_dbg.data);
    m_htcom_dbgOut.write();
    m_htrf.tm = m_qRef.tm;
    HumanSynchronizer::HRPPose3DToPose3D(hsp->hp_plot.getP("rf"),m_htrf_dbg.data);
    m_htrf_dbgOut.write();
    m_htlf.tm = m_qRef.tm;
    HumanSynchronizer::HRPPose3DToPose3D(hsp->hp_plot.getP("lf"),m_htlf_dbg.data);
    m_htlf_dbgOut.write();
    m_htrh.tm = m_qRef.tm;
    HumanSynchronizer::HRPPose3DToPose3D(hsp->hp_plot.getP("rh"),m_htrh_dbg.data);
    m_htrh_dbgOut.write();
    m_htlh.tm = m_qRef.tm;
    HumanSynchronizer::HRPPose3DToPose3D(hsp->hp_plot.getP("lh"),m_htlh_dbg.data);
    m_htlh_dbgOut.write();
    m_hthead.tm = m_qRef.tm;
    HumanSynchronizer::HRPPose3DToPose3D(hsp->hp_plot.getP("head"),m_hthead_dbg.data);
    m_hthead_dbgOut.write();
    m_htzmp.tm = m_qRef.tm;
    HumanSynchronizer::Vector3ToPoint3D(hsp->rp_ref_out.getP("zmp").p,m_rpzmp_dbg.data);
    m_htzmp_dbgOut.write();
    m_htrfw.tm = m_qRef.tm;
    m_htrfw_dbg.data.length(6);
    HumanSynchronizer::Wrench6ToDoubleSeq(hsp->hp_plot.getw("rfw"),m_htrfw_dbg.data);
    m_htrfw_dbgOut.write();
    m_htlfw.tm = m_qRef.tm;
    m_htlfw_dbg.data.length(6);
    HumanSynchronizer::Wrench6ToDoubleSeq(hsp->hp_plot.getw("lfw"),m_htlfw_dbg.data);
    m_htlfw_dbgOut.write();
    m_rpcom_dbg.tm = m_qRef.tm;
    HumanSynchronizer::HRPPose3DToPose3D(hsp->rp_ref_out.getP("com"),m_rpcom_dbg.data);
    m_rpcom_dbgOut.write();
    m_rprf_dbg.tm = m_qRef.tm;
    HumanSynchronizer::HRPPose3DToPose3D(hsp->rp_ref_out.getP("rf"),m_rprf_dbg.data);
    m_rprf_dbgOut.write();
    m_rplf_dbg.tm = m_qRef.tm;
    HumanSynchronizer::HRPPose3DToPose3D(hsp->rp_ref_out.getP("lf"),m_rplf_dbg.data);
    m_rplf_dbgOut.write();
    m_rprh_dbg.tm = m_qRef.tm;
    HumanSynchronizer::HRPPose3DToPose3D(hsp->rp_ref_out.getP("rh"),m_rprh_dbg.data);
    m_rprh_dbgOut.write();
    m_rplh_dbg.tm = m_qRef.tm;
    HumanSynchronizer::HRPPose3DToPose3D(hsp->rp_ref_out.getP("lh"),m_rplh_dbg.data);
    m_rplh_dbgOut.write();
    m_rphead_dbg.tm = m_qRef.tm;
    HumanSynchronizer::HRPPose3DToPose3D(hsp->rp_ref_out.getP("head"),m_rphead_dbg.data);
    m_rphead_dbgOut.write();
    m_rpzmp_dbg.tm = m_qRef.tm;
    HumanSynchronizer::Vector3ToPoint3D(hsp->rp_ref_out.getP("zmp").p,m_rpzmp_dbg.data);
    m_rpzmp_dbgOut.write();
    m_rpdcp_dbg.tm = m_qRef.tm;
    HumanSynchronizer::Vector3ToPoint3D(hsp->cp_dec,m_rpdcp_dbg.data);
    m_rpdcp_dbgOut.write();
    m_rpacp_dbg.tm = m_qRef.tm;
    HumanSynchronizer::Vector3ToPoint3D(hsp->cp_acc,m_rpacp_dbg.data);
    m_rpacp_dbgOut.write();
    m_invdyn_dbg.tm = m_qRef.tm;
    m_invdyn_dbg.data.length(6);
    HumanSynchronizer::Wrench6ToDoubleSeq(hsp->invdyn_ft,m_invdyn_dbg.data);
    m_invdyn_dbgOut.write();

    return RTC::RTC_OK;
}

void WholeBodyMasterSlave::processWholeBodyMasterSlave(){
  // Set ik target params
//  fik->target_root_p = target_root_p;
//  fik->target_root_R = target_root_R;
//  for ( std::map<std::string, SimpleFullbodyInverseKinematicsSolver::IKparam>::iterator it = fik->ikp.begin(); it != fik->ikp.end(); it++ ) {
//      it->second.target_p0 = ikp[it->first].target_p0;
//      it->second.target_r0 = ikp[it->first].target_r0;
//  }
//  fik->ratio_for_vel = transition_interpolator_ratio * leg_names_interpolator_ratio;
  fik->current_tm = m_qRef.tm;
//  for ( std::map<std::string, ABCIKparam>::iterator it = ikp.begin(); it != ikp.end(); it++ ) {
//      fik->ikp[it->first].is_ik_enable = it->second.is_active;
//  }

  fik->ikp["rleg"].is_ik_enable = true;
  fik->ikp["lleg"].is_ik_enable = true;
  fik->ikp["rarm"].is_ik_enable = true;
  fik->ikp["larm"].is_ik_enable = true;


//  // Revert
//  fik->revertRobotStateToCurrent();

  if(hsp->startCountdownForHumanSync){
    std::cerr << "[" << m_profile.instance_name << "] Count Down for HumanSync ["<<hsp->getRemainingCountDown()<<"]\r";
    hsp->updateCountDown();
  }
  if(hsp->isHumanSyncOn()){
    if(hsp->ht_first_call){
      std::cerr << "\n[" << m_profile.instance_name << "] Start HumanSync"<< std::endl;
      hsp->ht_first_call = false;
    }
  }else{
//    fik->revertRobotStateToCurrent();
    fik->storeCurrentParameters();
    fik->setReferenceJointAngles();
    hsp->setCurrentInputAsOffset(hsp->hp_wld_raw);
    hsp->calibInitHumanCOMFromZMP();

    hsp->rp_ref_out.getP("com").p_offs = m_robot->calcCM();
//    hsp->rp_ref_out.getP("zmp").p_offs = ref_zmp;
    const std::string robot_l_names[4] = {"rleg","lleg","rarm","larm"}, human_l_names[4] = {"rf","lf","rh","lh"};
    for(int i=0;i<4;i++){
      if(fik->ikp.count(robot_l_names[i])){
        hsp->rp_ref_out.getP(human_l_names[i]).p_offs = fik->ikp[robot_l_names[i]].target_link->p + fik->ikp[robot_l_names[i]].target_link->R * fik->ikp[robot_l_names[i]].localPos;
        hsp->rp_ref_out.getP(human_l_names[i]).rpy_offs = hrp::rpyFromRot(fik->ikp[robot_l_names[i]].target_link->R * fik->ikp[robot_l_names[i]].localR);
        hsp->rp_ref_out.getP(human_l_names[i]).p = hsp->rp_ref_out.getP(human_l_names[i]).p_offs;
        hsp->rp_ref_out.getP(human_l_names[i]).rpy = hsp->rp_ref_out.getP(human_l_names[i]).rpy_offs;
      }
    }
//
//    cout<<"r_"<<hsp->rp_ref_out.getP("rh").rpy.transpose()<<endl;
//    cout<<"r_offs"<<hsp->rp_ref_out.getP("rh").rpy_offs.transpose()<<endl;
//    cout<<"l_"<<hsp->rp_ref_out.getP("lh").rpy.transpose()<<endl;
//    cout<<"l_offs"<<hsp->rp_ref_out.getP("lh").rpy_offs.transpose()<<endl;


    hsp->pre_cont_rfpos = hsp->rp_ref_out.getP("rf").p_offs;
    hsp->pre_cont_lfpos = hsp->rp_ref_out.getP("lf").p_offs;
    hsp->baselinkpose.p_offs = m_robot->rootLink()->p;
    hsp->baselinkpose.rpy_offs = hrp::rpyFromRot(m_robot->rootLink()->R);
  }
  hrp::Vector3 rsole_pos = fik->ikp["rleg"].target_link->p+ fik->ikp["rleg"].target_link->R * fik->ikp["rleg"].localPos;
  hrp::Vector3 lsole_pos = fik->ikp["lleg"].target_link->p+ fik->ikp["lleg"].target_link->R * fik->ikp["lleg"].localPos;
  hsp->H_cur = m_robot->calcCM()(2) - std::min((double)rsole_pos(2), (double)lsole_pos(2));
  hsp->update();//////HumanSynchronizerの主要処理
  if(loop%100==0)hsp->rp_ref_out.print();
  m_robot->calcForwardKinematics();
  // additional COM fitting IK for HumanSynchronizer
  if(hsp->isHumanSyncOn()){

//    for(int i=0;i<m_robot->numJoints();i++){ cout <<m_robot->joint(i)->name<<" "<<m_robot->joint(i)->q<<endl; }
    solveFullbodyIKStrictCOM( hsp->rp_ref_out.getP("com"), hsp->rp_ref_out.getP("rf"), hsp->rp_ref_out.getP("lf"), hsp->rp_ref_out.getP("rh"), hsp->rp_ref_out.getP("lh"), hsp->cam_rpy_filtered );

//    for(int i=0;i<m_robot->numJoints();i++){ cout <<m_robot->joint(i)->name<<" "<<m_robot->joint(i)->q<<endl; }
//    exit();
    //outport用のデータ上書き
//    ref_zmp = hsp->rp_ref_out.getP("zmp").p;
//    ref_cog = hsp->rp_ref_out.getP("com").p;

    rel_ref_zmp = m_robot->rootLink()->R.transpose() * (hsp->rp_ref_out.getP("zmp").p - m_robot->rootLink()->p);

//    m_contactStates.data[contact_states_index_map["rleg"]] = hsp->is_rf_contact;
//    m_contactStates.data[contact_states_index_map["lleg"]] = hsp->is_lf_contact;
//
//
//    for ( std::map<std::string, ABCIKparam>::iterator it = fik->ikp.begin(); it != fik->ikp.end(); it++ ) {
//        m_contactStates.data[contact_states_index_map[it->first]] = isOptionalDataContact(it->first);
//        m_controlSwingSupportTime.data[contact_states_index_map[it->first]] = m_optionalData.data[contact_states_index_map[it->first]+contact_states_index_map.size()];
//    }
//
    m_optionalData2.data[contact_states_index_map["rleg"]] = hsp->is_rf_contact;
    m_optionalData2.data[contact_states_index_map["lleg"]] = hsp->is_lf_contact;


//    for(int i=0;i<m_optionalData2.data.length();i++){
//      cout<<"m_optionalData2.data["<<i<<"]:"<<m_optionalData2.data[i]<<endl;
//    }

//        cout<<"rleg"<<m_optionalData.data[contact_states_index_map["rleg"]]<<endl;
//        cout<<"lleg"<<m_optionalData.data[contact_states_index_map["lleg"]]<<endl;

  }else{
    rel_ref_zmp = m_robot->rootLink()->R.transpose() * (hsp->rp_ref_out.getP("zmp").p - m_robot->rootLink()->p);
  }

//  cout<<"len2:"<<m_optionalData2.data.length()<<endl;





//  for(int i=0;i<m_robot->numJoints();i++){ cout <<m_robot->joint(i)->name<<" "<<m_robot->joint(i)->q<<endl; }
}
//
//
//hrp::Vector3 torso_pos = hrp::Vector3::Zero();
//hrp::Vector3 torso_rot = hrp::Vector3::Zero();
////void WholeBodyMasterSlave::calcDynamicsFilterCompensation(const hrp::Vector3 zmp_lip, const hrp::Vector3 zmp_fullbody){
////  hrp::Vector3 torso_pos_acc,torso_rot_acc;
////  static hrp::Vector3 torso_pos_vel,torso_rot_vel = hrp::Vector3::Zero();
////  hrp::Matrix33 total_inertia;
////  for(int i=0; i<m_robot->numLinks(); i++){
////    total_inertia += m_robot->link(i)->I;
////  }
////  hrp::Vector3 zmp_err = zmp_fullbody - zmp_lip;
////  if(fabs(zmp_err(0))<0.002)zmp_err(0)=0;
////  if(fabs(zmp_err(1))<0.002)zmp_err(1)=0;
////  LIMIT_MINMAX(zmp_err(0),-0.05,0.05);
////  LIMIT_MINMAX(zmp_err(1),-0.05,0.05);
////  zmp_err(2)=0;
////  torso_pos_acc(0) = zmp_err(0) / 1.1 * 9.8;
////  torso_pos_acc(1) = zmp_err(1) / 1.1 * 9.8;
////  torso_rot_acc(0) = - zmp_err(1) * m_robot->totalMass() * 9.8 / (130*0.2*0.2);
////  torso_rot_acc(1) = + zmp_err(0) * m_robot->totalMass() * 9.8 / (130*0.2*0.2);
////
////  torso_pos_acc(0) *= 0.5;
////  torso_pos_acc(1) *= 0.5;
////  torso_rot_acc(0) *= 0.1;
////  torso_rot_acc(1) *= 0.1;
////
////
////
////  torso_pos_acc -= 1 * torso_pos + 1 * torso_pos_vel;
////  torso_rot_acc -= 1 * torso_rot + 1 * torso_rot_vel;
//////  torso_rot_acc(0) -= 10 * torso_rot(0) + 10 * torso_rot_vel(0);
//////  torso_rot_acc(1) -= 10 * torso_rot(1) + 10 * torso_rot_vel(1);
////
////  LIMIT_MINMAX(torso_pos_acc(0),-10,10);
////  LIMIT_MINMAX(torso_pos_acc(1),-10,10);
////  LIMIT_MINMAX(torso_pos_acc(2),-0,0);
////  LIMIT_MINMAX(torso_rot_acc(0),-10,10);
////  LIMIT_MINMAX(torso_rot_acc(1),-10,10);
////  LIMIT_MINMAX(torso_rot_acc(2),-0,0);
////
////
////  if(hsp->isHumanSyncOn()){
////    torso_rot_vel += torso_rot_acc * m_dt;
////    torso_rot += torso_rot_vel * m_dt;
////    torso_pos_vel += torso_pos_acc * m_dt;
////    torso_pos += torso_pos_vel * m_dt;
////  }
////
////  LIMIT_MINMAX(torso_pos(0),-0.1,0.1);
////  LIMIT_MINMAX(torso_pos(1),-0.1,0.1);
////  LIMIT_MINMAX(torso_pos(2),-0.1,0.1);
////  LIMIT_MINMAX(torso_rot(0),-10*M_PI/180,10*M_PI/180);
////  LIMIT_MINMAX(torso_rot(1),-10*M_PI/180,10*M_PI/180);
////  LIMIT_MINMAX(torso_rot(2),-0.1,0.1);
////
////
//////  cout<<"total_inertia: "<<total_inertia<<endl;
//////  cout<<"zmp_lip: "<<zmp_lip.transpose()<<endl;
//////  cout<<"zmp_fullbody: "<<zmp_fullbody.transpose()<<endl;
//////  cout<<"torso_rot_acc: "<<torso_rot_acc.transpose()<<endl;
//////  cout<<"torso_rot_vel: "<<torso_rot_vel.transpose()<<endl;
//////  cout<<"torso_rot: "<<torso_rot.transpose()<<endl;
////
////
////  fprintf(hsp->id_log,"zmp_lip: %f %f ",zmp_lip(0),zmp_lip(1),zmp_lip(2));
////  fprintf(hsp->id_log,"zmp_fullbody: %f %f ",zmp_fullbody(0),zmp_fullbody(1),zmp_fullbody(2));
////  fprintf(hsp->id_log,"zmp_fullbody2: %f %f ",ref_zmp_invdyn2(0),ref_zmp_invdyn2(1),ref_zmp_invdyn2(2));
////  fprintf(hsp->id_log,"torso_pos_acc: %f %f ",torso_pos_acc(0),torso_pos_acc(1),torso_pos_acc(2));
////  fprintf(hsp->id_log,"torso_pos: %f %f ",torso_pos(0),torso_pos(1),torso_pos(2));
////  fprintf(hsp->id_log,"torso_rot_acc: %f %f ",torso_rot_acc(0),torso_rot_acc(1),torso_rot_acc(2));
////  fprintf(hsp->id_log,"torso_rot: %f %f ",torso_rot(0),torso_rot(1),torso_rot(2));
////  fprintf(hsp->id_log,"\n");
////
////
////
//////  fik->ratio_for_vel = transition_interpolator_ratio * leg_names_interpolator_ratio;
//////  fik->current_tm = m_qRef.tm;
//////  for ( std::map<std::string, ABCIKparam>::iterator it = ikp.begin(); it != ikp.end(); it++ ) {
//////      fik->ikp[it->first].is_ik_enable = it->second.is_active;
//////  }
//////  if(hsp->isHumanSyncOn()){
//////    if(hsp->ht_first_call){
//////      std::cerr << "\n[" << m_profile.instance_name << "] Start HumanSync"<< std::endl;
//////      hsp->ht_first_call = false;
//////    }
//////  }
//////  m_robot->calcForwardKinematics();
//////  // additional COM fitting IK for HumanSynchronizer
////  if(hsp->isHumanSyncOn()){
////    HRPPose3D com_mod = hsp->rp_ref_out.getP("com");
////    com_mod.p += torso_pos;
////    com_mod.rpy += torso_rot;
////    solveFullbodyIKStrictCOM( com_mod, hsp->rp_ref_out.getP("rf"), hsp->rp_ref_out.getP("lf"), hsp->rp_ref_out.getP("rh"), hsp->rp_ref_out.getP("lh"), hsp->cam_rpy_filtered );
////  }
////}
//
//
//void WholeBodyMasterSlave::calcDynamicsFilterCompensation(const hrp::Vector3 zmp_lip, const hrp::Vector3 zmp_fullbody){
//
//
//
////  fik->ratio_for_vel = transition_interpolator_ratio * leg_names_interpolator_ratio;
////  fik->current_tm = m_qRef.tm;
////  for ( std::map<std::string, ABCIKparam>::iterator it = ikp.begin(); it != ikp.end(); it++ ) {
////      fik->ikp[it->first].is_ik_enable = it->second.is_active;
////  }
//////  // Revert
////  fik->revertRobotStateToCurrent();
//
////  hrp::dmatrix Jp;
////  hrp::dmatrix Jl;
////  m_robot->calcCM();
////  m_robot->calcCMJacobian(NULL, Jp); // Eq.1 upper [M/m E -r]
////  m_robot->calcAngularMomentumJacobian(NULL, Jl); // Eq.1 lower [H 0 I]
////  Jp *= m_robot->totalMass();
////  hrp::dmatrix Jpl(Jp.rows() + Jl.rows(), Jp.cols());
////  Jpl << Jp, Jl;
////  hrp::dvector all_vel_vec(3+3+m_robot->numJoints());
////  all_vel_vec << idsb.base_v, idsb.base_w, idsb.dq;
//  hrp::Vector3 P,L;
//  m_robot->rootLink()->v = idsb.base_v;
//  m_robot->rootLink()->w = idsb.base_w;
//  m_robot->calcTotalMomentum(P,L);
//  L = L - m_robot->calcCM().cross(P);
//
//  torso_rot(0) -= L(0) * 0.1 * m_dt;
//  torso_rot(1) -= L(1) * 0.1 * m_dt;
//  torso_rot(2) -= L(2) * 0.1 * m_dt;
//
//
//
//  fprintf(hsp->id_log,"p: %f %f ",hsp->rp_ref_out.getP("com").p(0),hsp->rp_ref_out.getP("com").p(1),hsp->rp_ref_out.getP("com").p(2));
//  fprintf(hsp->id_log,"rpy: %f %f ",hsp->rp_ref_out.getP("com").rpy(0),hsp->rp_ref_out.getP("com").rpy(1),hsp->rp_ref_out.getP("com").rpy(2));
//  fprintf(hsp->id_log,"P: %f %f ",P(0),P(1),P(2));
//  fprintf(hsp->id_log,"L: %f %f ",L(0),L(1),L(2));
////  fprintf(hsp->id_log,"p_v: %f %f ",idsb.base_v(0),idsb.base_v(1),idsb.base_v(2));
////  fprintf(hsp->id_log,"rpy_v: %f %f ",idsb.base_w(0),idsb.base_w(1),idsb.base_w(2));
//    fprintf(hsp->id_log,"torso_pos: %f %f ",torso_pos(0),torso_pos(1),torso_pos(2));
//    fprintf(hsp->id_log,"torso_rot: %f %f ",torso_rot(0),torso_rot(1),torso_rot(2));
//  fprintf(hsp->id_log,"\n");
//
//  if(hsp->isHumanSyncOn()){
//    HRPPose3D com_mod = hsp->rp_ref_out.getP("com");
////    com_mod.p += torso_pos;
////    com_mod.rpy += torso_rot;
//    solveFullbodyIKStrictCOM( com_mod, hsp->rp_ref_out.getP("rf"), hsp->rp_ref_out.getP("lf"), hsp->rp_ref_out.getP("rh"), hsp->rp_ref_out.getP("lh"), hsp->cam_rpy_filtered );
//  }
//}
//
//
//

void WholeBodyMasterSlave::solveFullbodyIKStrictCOM(const HRPPose3D& com_ref, const HRPPose3D& rf_ref, const HRPPose3D& lf_ref, const HRPPose3D& rh_ref, const HRPPose3D& lh_ref, const hrp::Vector3& head_ref){
  int com_ik_loop=0;
  const int COM_IK_MAX_LOOP = 10;
  const double COM_IK_MAX_ERROR = 1e-4;

//  cout<<""<<m_robot->rootLink()->p.transpose()<<" : "<<com_ref.p.transpose()<<endl;

  m_robot->rootLink()->p = com_ref.p;//move base link at first
//  m_robot->rootLink()->p = com_ref.p + hsp->torso_rot;//move base link at first
  m_robot->rootLink()->R = hrp::rotFromRpy(com_ref.rpy);//move base link at first

  //////////invdyncomp
//  m_robot->rootLink()->R *= hrp::rotFromRpy(hsp->torso_rot);
//  m_robot->rootLink()->p += torso_rot;

  hrp::Vector3 tmp_com_err;
  const std::string names[4] = {"rleg","lleg","rarm","larm"};
  const HRPPose3D* refs[4] = {&rf_ref, &lf_ref, &rh_ref, &lh_ref};
  for(int i=0;i<4;i++){
    if(fik->ikp.count(names[i])){
      fik->ikp[names[i]].target_r0 = hrp::rotFromRpy(refs[i]->rpy);//convert End Effector pos into link origin pos
      fik->ikp[names[i]].target_p0 = refs[i]->p;
    }
  }
  if(m_robot->link("HEAD_JOINT0") != NULL)m_robot->joint(15)->q = head_ref(2);
  if(m_robot->link("HEAD_JOINT1") != NULL)m_robot->joint(16)->q = head_ref(1);
  //COM 収束ループ
  while(
      m_robot->calcForwardKinematics(),
      tmp_com_err = com_ref.p - m_robot->calcCM(),
      tmp_com_err.norm() > COM_IK_MAX_ERROR)
  {
    m_robot->rootLink()->p += tmp_com_err;
    m_robot->calcForwardKinematics();
    for ( std::map<std::string, SimpleFullbodyInverseKinematicsSolver::IKparam>::iterator it = fik->ikp.begin(); it != fik->ikp.end(); it++ ) {
        if (it->second.is_ik_enable) fik->solveLimbIK (it->second, it->first, fik->ratio_for_vel, false);
    }
    if(com_ik_loop++ > COM_IK_MAX_LOOP){std::cerr << "COM constraint IK MAX loop [="<<COM_IK_MAX_LOOP<<"] exceeded!!" << std::endl; break; };
  }
//  cout<<"COM_IK_MAX_LOOP:"<<com_ik_loop<<endl;
}

bool WholeBodyMasterSlave::startCountDownForWholeBodyMasterSlave(const double sec)
{
  if(sec >= 0.0 && sec <= 30.0){
    std::cerr << "[" << m_profile.instance_name << "] start Synchronization after "<<sec<<" [s]" << std::endl;
    hsp->countdown_sec = sec;
    hsp->startCountdownForHumanSync = true;
    return true;
  }else{
    std::cerr << "[" << m_profile.instance_name << "] Count Down Time must be 0 < T < 30 [s]"<< std::endl;
    return false;
  }
}

bool WholeBodyMasterSlave::stopHumanSync()
{
	std::cerr << "[" << m_profile.instance_name << "] stop HumanSync Now" << std::endl;
	return true;
}

bool WholeBodyMasterSlave::setWholeBodyMasterSlaveParam(const OpenHRP::WholeBodyMasterSlaveService::WholeBodyMasterSlaveParam& i_param){
  std::cerr << "[" << m_profile.instance_name << "] setWholeBodyMasterSlaveParam" << std::endl;
  hsp->WBMSparam.use_rh = hsp->WBMSparam.use_lh = i_param.use_hands;
  hsp->WBMSparam.use_head = i_param.use_head;
  hsp->WBMSparam.set_com_height_fix = i_param.set_com_height_fix;
  hsp->WBMSparam.set_com_height_fix_val = i_param.set_com_height_fix_val;
  hsp->WBMSparam.foot_vertical_vel_limit_coeff = i_param.foot_vertical_vel_limit_coeff;
  hsp->WBMSparam.human_com_height = i_param.human_com_height;
  hsp->WBMSparam.swing_foot_height_offset = i_param.swing_foot_height_offset;
  hsp->WBMSparam.swing_foot_max_height = i_param.swing_foot_max_height;
  hsp->WBMSparam.auto_swing_foot_landing_threshold = i_param.auto_swing_foot_landing_threshold;
  return true;
}
bool WholeBodyMasterSlave::getWholeBodyMasterSlaveParam(OpenHRP::WholeBodyMasterSlaveService::WholeBodyMasterSlaveParam& i_param){
  std::cerr << "[" << m_profile.instance_name << "] getWholeBodyMasterSlaveParam" << std::endl;
  i_param.set_com_height_fix = hsp->WBMSparam.set_com_height_fix;
  i_param.set_com_height_fix_val = hsp->WBMSparam.set_com_height_fix_val;
  i_param.foot_vertical_vel_limit_coeff = hsp->WBMSparam.foot_vertical_vel_limit_coeff;
  i_param.human_com_height = hsp->WBMSparam.human_com_height;
  i_param.use_hands = (hsp->WBMSparam.use_rh || hsp->WBMSparam.use_lh);
  i_param.use_head = hsp->WBMSparam.use_head;
  i_param.swing_foot_height_offset = hsp->WBMSparam.swing_foot_height_offset;
  i_param.swing_foot_max_height = hsp->WBMSparam.swing_foot_max_height;
  i_param.auto_swing_foot_landing_threshold = hsp->WBMSparam.auto_swing_foot_landing_threshold;
  return true;
}

//
extern "C"
{

    void WholeBodyMasterSlaveInit(RTC::Manager* manager)
    {
        RTC::Properties profile(WholeBodyMasterSlave_spec);
        manager->registerFactory(profile,
                                 RTC::Create<WholeBodyMasterSlave>,
                                 RTC::Delete<WholeBodyMasterSlave>);
    }

};


