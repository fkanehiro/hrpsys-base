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


#define MAX_TRANSITION_COUNT (2/m_dt)
typedef coil::Guard<coil::Mutex> Guard;
using namespace rats;

// Module specification
// <rtc-template block="module_spec">
static const char* autobalancer_spec[] =
    {
        "implementation_id", "AutoBalancer",
        "type_name",         "AutoBalancer",
        "description",       "autobalancer component",
        "version",           "1.0",
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
      m_qCurrentIn("qCurrent", m_qCurrent),
      m_qOut("q", m_q),
      m_zmpRefOut("zmpRef", m_zmpRef),
      m_basePosOut("basePos", m_basePos),
      m_baseRpyOut("baseRpy", m_baseRpy),
      m_AutoBalancerServicePort("AutoBalancerService"),
      // </rtc-template>
      move_base_gain(0.1),
      m_robot(hrp::BodyPtr()),
      m_debugLevel(0),
      dummy(0)
{
    m_service0.autobalancer(this);
}

AutoBalancer::~AutoBalancer()
{
}


RTC::ReturnCode_t AutoBalancer::onInitialize()
{
    std::cout << "AutoBalancer::onInitialize()" << std::endl;
    bindParameter("debugLevel", m_debugLevel, "0");

    // Registration: InPort/OutPort/Service
    // <rtc-template block="registration">
    // Set InPort buffers
    addInPort("qRef", m_qRefIn);
    addInPort("qCurrent", m_qCurrentIn);

    // Set OutPort buffer
    addOutPort("q", m_qOut);
    addOutPort("zmpRef", m_zmpRefOut);
    addOutPort("basePos", m_basePosOut);
    addOutPort("baseRpy", m_baseRpyOut);
  
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
      std::cerr << "failed to load model[" << prop["model"] << "] in "
                << m_profile.instance_name << std::endl;
      return RTC::RTC_ERROR;
    }

    // allocate memory for outPorts
    m_qRef.data.length(m_robot->numJoints());
    m_qCurrent.data.length(m_robot->numJoints());
    m_q.data.length(m_robot->numJoints());
    qorg.resize(m_robot->numJoints());
    qrefv.resize(m_robot->numJoints());

    transition_count = 0;
    control_mode = MODE_IDLE;
    loop = 0;

    // setting from conf file
    // GaitGenerator requires abc_leg_offset and abc_stride_parameter in robot conf file
    // setting leg_pos from conf file
    coil::vstring leg_offset_str = coil::split(prop["abc_leg_offset"], ",");
    std::vector<hrp::Vector3> leg_pos;
    if (leg_offset_str.size() > 0) {
      hrp::Vector3 leg_offset;
      for (size_t i = 0; i < 3; i++) coil::stringTo(leg_offset(i), leg_offset_str[i].c_str());
      std::cerr << "[AutoBalancer] abc_leg_offset : " << leg_offset(0) << " " << leg_offset(1) << " " << leg_offset(2) << std::endl;
      leg_pos.push_back(hrp::Vector3(-1*leg_offset));
      leg_pos.push_back(hrp::Vector3(leg_offset));
    }
    // setting stride limitations from conf file
    coil::vstring stride_param_str = coil::split(prop["abc_stride_parameter"], ",");
    hrp::Vector3 stride_param;
    if (stride_param_str.size() > 0) {
      for (size_t i = 0; i < 3; i++) coil::stringTo(stride_param(i), stride_param_str[i].c_str());
      std::cerr << "[AutoBalancer] abc_stride_parameter : " << stride_param(0) << " " << stride_param(1) << " " << stride_param(2) << std::endl;
    }
    if (default_zmp_offsets.size() == 0) {
      for (size_t i = 0; i < 2; i++) default_zmp_offsets.push_back(hrp::Vector3::Zero());
    }
    if (leg_offset_str.size() > 0 && stride_param_str.size() > 0) {
      gg = ggPtr(new rats::gait_generator(m_dt, leg_pos, stride_param(0)/*[m]*/, stride_param(1)/*[m]*/, stride_param(2)/*[deg]*/));
      gg->set_default_zmp_offsets(default_zmp_offsets);
    }
    gg_is_walking = gg_ending = gg_solved = false;
    fix_leg_coords = coordinates();

    // setting from conf file
    // :rleg,TARGET_LINK,BASE_LINK
    coil::vstring end_effectors_str = coil::split(prop["abc_end_effectors"], ",");
    if (end_effectors_str.size() > 0) {
      size_t num = end_effectors_str.size()/3;
      for (size_t i = 0; i < num; i++) {
        std::string ee_name, ee_target, ee_base;
        coil::stringTo(ee_name, end_effectors_str[i*3].c_str());
        coil::stringTo(ee_target, end_effectors_str[i*3+1].c_str());
        coil::stringTo(ee_base, end_effectors_str[i*3+2].c_str());
        std::cerr << "abc limb[" << ee_name << "] " << ee_target << " " << ee_base << std::endl;
        ABCIKparam tp;
        tp.base_name = ee_base;
        tp.target_name = ee_target;
        tp.manip = hrp::JointPathExPtr(new hrp::JointPathEx(m_robot, m_robot->link(tp.base_name),
                                                            m_robot->link(tp.target_name)));
        ikp.insert(std::pair<std::string, ABCIKparam>(ee_name , tp));
      }
    }

    // ref force port
    coil::vstring virtual_force_sensor = coil::split(prop["virtual_force_sensor"], ",");
    int npforce = m_robot->numSensors(hrp::Sensor::FORCE);
    int nvforce = virtual_force_sensor.size()/10;
    int nforce  = npforce + nvforce;
    m_ref_force.resize(nforce);
    m_ref_forceIn.resize(nforce);
    for (unsigned int i=0; i<npforce; i++){
        hrp::Sensor *s = m_robot->sensor(hrp::Sensor::FORCE, i);
        m_ref_forceIn[i] = new InPort<TimedDoubleSeq>(std::string("ref_"+s->name).c_str(), m_ref_force[i]);
        m_ref_force[i].data.length(6);
        registerInPort(std::string("ref_"+s->name).c_str(), *m_ref_forceIn[i]);
        std::cerr << s->name << std::endl;
    }
    for (unsigned int i=0; i<nvforce; i++){
        std::string name = virtual_force_sensor[i*10+0];
        m_ref_forceIn[i+npforce] = new InPort<TimedDoubleSeq>(std::string("ref_"+name).c_str(), m_ref_force[i+npforce]);
        m_ref_force[i+npforce].data.length(6);
        registerInPort(std::string("ref_"+name).c_str(), *m_ref_forceIn[i+npforce]);
    }
    for (unsigned int i=0; i<m_ref_forceIn.size(); i++){
      ref_forces.push_back(hrp::Vector3(0,0,0));
    }
    sbp_offset = hrp::Vector3(0,0,0);
    sbp_cog_offset = hrp::Vector3(0,0,0);
    //use_force = MODE_NO_FORCE;
    use_force = MODE_REF_FORCE;

    return RTC::RTC_OK;
}



RTC::ReturnCode_t AutoBalancer::onFinalize()
{
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
    std::cout << "AutoBalancer::onActivated(" << ec_id << ")" << std::endl;
    
    return RTC::RTC_OK;
}

/*
  RTC::ReturnCode_t AutoBalancer::onDeactivated(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

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
    if (m_qCurrentIn.isNew()) {
        m_qCurrentIn.read();
        is_qCurrent = true;
    } else {
      is_qCurrent = false;
    }
    for (unsigned int i=0; i<m_ref_forceIn.size(); i++){
        if ( m_ref_forceIn[i]->isNew() ) {
            m_ref_forceIn[i]->read();
        }
    }
    Guard guard(m_mutex);
    robotstateOrg2qRef();
    if (control_mode == MODE_ABC ) solveLimbIK();
    if ( m_q.data.length() != 0 ) { // initialized
      for ( int i = 0; i < m_robot->numJoints(); i++ ){
        m_q.data[i] = m_robot->joint(i)->q;
      }
      m_qOut.write();
    }
    hrp::Vector3 baseRpy = hrp::rpyFromRot(m_robot->rootLink()->R);
    m_baseRpy.data.r = baseRpy(0);
    m_baseRpy.data.p = baseRpy(1);
    m_baseRpy.data.y = baseRpy(2);
    m_baseRpyOut.write();
    m_basePos.data.x = m_robot->rootLink()->p(0);
    m_basePos.data.y = m_robot->rootLink()->p(1);
    m_basePos.data.z = m_robot->rootLink()->p(2);
    m_basePosOut.write();
    m_zmpRef.data.x = refzmp(0);
    m_zmpRef.data.y = refzmp(1);
    m_zmpRef.data.z = refzmp(2);
    m_zmpRefOut.write();
    return RTC::RTC_OK;
}

void AutoBalancer::robotstateOrg2qRef()
{
  base_pos_org = m_robot->rootLink()->p;
  base_rot_org = m_robot->rootLink()->R;
  for ( int i = 0; i < m_robot->numJoints(); i++ ){
    qorg[i] = m_robot->joint(i)->q;
    m_robot->joint(i)->q = m_qRef.data[i];
    qrefv[i] = m_qRef.data[i];
  }
  m_robot->calcForwardKinematics();
  if ( ikp.find(":rleg") != ikp.end() && ikp.find(":lleg") != ikp.end() ) {
    coordinates tmp_fix_coords;
    if ( gg_is_walking ) {
      gg->set_default_zmp_offsets(default_zmp_offsets);
      gg_solved = gg->proc_one_tick(rats::gait_generator::CYCLOID);
      ikp[gg->get_support_leg()].target_p0 = gg->get_support_leg_coords().pos;
      ikp[gg->get_support_leg()].target_r0 = gg->get_support_leg_coords().rot;
      ikp[gg->get_swing_leg()].target_p0 = gg->get_swing_leg_coords().pos;
      ikp[gg->get_swing_leg()].target_r0 = gg->get_swing_leg_coords().rot;
      gg->get_swing_support_mid_coords(tmp_fix_coords);
    } else {
      tmp_fix_coords = fix_leg_coords;
    }
    fixLegToCoords(":both", tmp_fix_coords);

    /* update ref_forces ;; sp's absolute -> rmc's absolute */
    for (size_t i = 0; i < m_ref_forceIn.size(); i++)
      tmp_fix_coords.rotate_vector(ref_forces[i],
                                   hrp::Vector3(m_ref_force[i].data[0], m_ref_force[i].data[1], m_ref_force[i].data[2]));
    tmp_fix_coords.rotate_vector(sbp_offset, hrp::Vector3(sbp_offset));

    target_base_pos = m_robot->rootLink()->p;
    target_base_rot = m_robot->rootLink()->R;
    if (!gg_is_walking) {
      for ( std::map<std::string, ABCIKparam>::iterator it = ikp.begin(); it != ikp.end(); it++ ) {
        it->second.target_p0 = m_robot->link(it->second.target_name)->p;
        it->second.target_r0 = m_robot->link(it->second.target_name)->R;
      }
    }
    if (gg_is_walking) {
      target_com = gg->get_cog();
    } else {
      //coordinates rc(target_coords[":rleg"]), lc(target_coords[":lleg"]);
      //rc.translate(tmpzo[0]); /* :rleg */
      //lc.translate(tmpzo[1]); /* :lleg */
      //target_cog = (rc.pos + lc.pos) / 2.0;
      //target_com = hrp::Vector3::Zero();
      coordinates rc(m_robot->link(ikp[":rleg"].target_name)->p,
                     m_robot->link(ikp[":rleg"].target_name)->R);
      coordinates lc(m_robot->link(ikp[":lleg"].target_name)->p,
                     m_robot->link(ikp[":lleg"].target_name)->R);
      rc.translate(default_zmp_offsets[0]); /* :rleg */
      lc.translate(default_zmp_offsets[1]); /* :lleg */
      target_com = (rc.pos+lc.pos)/2.0;
    }
  }
  if ( transition_count > 0 ) {
    double transition_smooth_gain = 1/(1+exp(-9.19*(((MAX_TRANSITION_COUNT - transition_count) / MAX_TRANSITION_COUNT) - 0.5)));
    for ( std::map<std::string, ABCIKparam>::iterator it = ikp.begin(); it != ikp.end(); it++ ) {
      hrp::JointPathExPtr manip = it->second.manip;
      for ( int j = 0; j < manip->numJoints(); j++ ) {
        int i = manip->joint(j)->jointId; // index in robot model
        hrp::Link* joint =  m_robot->joint(i);
        // transition_smooth_gain moves from 0 to 1
        // (/ (log (/ (- 1 0.99) 0.99)) 0.5)
        joint->q = ( m_qRef.data[i] - transition_joint_q[i] ) * transition_smooth_gain + transition_joint_q[i];
      }
    }
    transition_count--;
    if(transition_count <= 0){ // erase impedance param
      std::cerr << "Finished cleanup" << std::endl;
      control_mode = MODE_IDLE;
    }
  }
  if (control_mode == MODE_IDLE) {
    //refzmp = hrp::Vector3(0,0,0); // tempolary
    if ( ikp.find(":rleg") != ikp.end() && ikp.find(":lleg") != ikp.end() )
      refzmp = (m_robot->link(ikp[":rleg"].target_name)->p+
                m_robot->link(ikp[":lleg"].target_name)->p)/2.0;
    else refzmp = hrp::Vector3(0,0,0);
  } else if (gg_is_walking) {
    refzmp = gg->get_refzmp();
  } else {
    if ( ikp.find(":rleg") != ikp.end() && ikp.find(":lleg") != ikp.end() )
      refzmp = target_com;
    else refzmp = hrp::Vector3(0,0,0);
  }
  if ( transition_count > 0 ) {
    double transition_smooth_gain = 1/(1+exp(-9.19*(((MAX_TRANSITION_COUNT - abs(transition_count)) / MAX_TRANSITION_COUNT) - 0.5)));
    refzmp = transition_smooth_gain * ( refzmp - prefzmp ) + prefzmp;
  }
}

void AutoBalancer::fixLegToCoords (const std::string& leg, const coordinates& coords)
{
  coordinates tar, ref, delta, tmp;
  coordinates rleg_endcoords(m_robot->link(ikp[":rleg"].target_name)->p,
                             m_robot->link(ikp[":rleg"].target_name)->R);
  coordinates lleg_endcoords(m_robot->link(ikp[":lleg"].target_name)->p,
                             m_robot->link(ikp[":lleg"].target_name)->R);
  mid_coords(tar, 0.5, rleg_endcoords , lleg_endcoords);
  tmp = coords;
  ref = coordinates(m_robot->rootLink()->p, m_robot->rootLink()->R);
  tar.transformation(delta, ref, ":local");
  tmp.transform(delta, ":local");
  m_robot->rootLink()->p = tmp.pos;
  m_robot->rootLink()->R = tmp.rot;
  m_robot->calcForwardKinematics();
}

bool AutoBalancer::solveLimbIKforLimb (ABCIKparam& param, const double transition_smooth_gain)
{
  param.current_p0 = m_robot->link(param.target_name)->p;
  param.current_r0 = m_robot->link(param.target_name)->R;

  hrp::Vector3 vel_p, vel_r;
  vel_p = param.target_p0 - param.current_p0;
  rats::difference_rotation(vel_r, param.current_r0, param.target_r0);
  param.manip->solveLimbIK(vel_p, vel_r, transition_count, 0.001, 0.01, MAX_TRANSITION_COUNT, qrefv, DEBUGP);
  return true;
}

void AutoBalancer::solveLimbIK ()
{
  double transition_smooth_gain = 1.0;
  if ( transition_count < 0 ) {
    // (/ (log (/ (- 1 0.99) 0.99)) 0.5)
    transition_smooth_gain = 1/(1+exp(-9.19*(((MAX_TRANSITION_COUNT + transition_count) / MAX_TRANSITION_COUNT) - 0.5)));
  }

  for ( std::map<std::string, ABCIKparam>::iterator it = ikp.begin(); it != ikp.end(); it++ ) {
    if (it->second.is_active) {
      for ( int j = 0; j < it->second.manip->numJoints(); j++ ){
	int i = it->second.manip->joint(j)->jointId;
	m_robot->joint(i)->q = qorg[i];
      }
    }
  }
  m_robot->rootLink()->p = base_pos_org;
  m_robot->rootLink()->R = base_rot_org;
  m_robot->calcForwardKinematics();
  hrp::Vector3 tmp_input_sbp = hrp::Vector3(0,0,0);
  static_balance_point_proc_one(tmp_input_sbp, refzmp(2));
  hrp::Vector3 dif_com = tmp_input_sbp - target_com;
  dif_com(2) = m_robot->rootLink()->p(2) - target_base_pos(2);
  m_robot->rootLink()->p = m_robot->rootLink()->p + -1 * move_base_gain * transition_smooth_gain * dif_com;
  m_robot->rootLink()->R = target_base_rot;
  m_robot->calcForwardKinematics();

  for ( std::map<std::string, ABCIKparam>::iterator it = ikp.begin(); it != ikp.end(); it++ ) {
    if (it->second.is_active) solveLimbIKforLimb(it->second, transition_smooth_gain);
  }
  if ( transition_count < 0 ) {
    transition_count++;
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

void AutoBalancer::startABCparam(const OpenHRP::AutoBalancerService::AutoBalancerLimbParamSequence& alp)
{
  std::cerr << "[AutoBalancer] start auto balancer mode" << std::endl;
  transition_count = -MAX_TRANSITION_COUNT; // when start impedance, count up to 0
  Guard guard(m_mutex);
  for ( std::map<std::string, ABCIKparam>::iterator it = ikp.begin(); it != ikp.end(); it++ ) {
    it->second.is_active = false;
  }

  for (size_t i = 0; i < alp.length(); i++) {
    const OpenHRP::AutoBalancerService::AutoBalancerLimbParam& tmpalp = alp[i];
    ABCIKparam& tmp = ikp[std::string(tmpalp.name)];
    memcpy(tmp.target2foot_offset_pos.data(), tmpalp.target2foot_offset_pos, sizeof(double)*3);
    tmp.target2foot_offset_rot = (Eigen::Quaternion<double>(tmpalp.target2foot_offset_rot[0],
                                                            tmpalp.target2foot_offset_rot[1],
                                                            tmpalp.target2foot_offset_rot[2],
                                                            tmpalp.target2foot_offset_rot[3])).normalized().toRotationMatrix();
    tmp.is_active = true;
    std::cerr << "abc limb [" << std::string(tmpalp.name) << "]" << std::endl;
    std::cerr << "     offset_pos : " << tmp.target2foot_offset_pos(0) << " " << tmp.target2foot_offset_pos(1) << " " << tmp.target2foot_offset_pos(2) << std::endl;
  }

  for ( int i = 0; i < m_robot->numJoints(); i++ ){
    m_robot->joint(i)->q = m_qRef.data[i];
  }
  m_robot->calcForwardKinematics();
  fixLegToCoords(":both", fix_leg_coords);
  base_pos_org = m_robot->rootLink()->p;
  base_rot_org = m_robot->rootLink()->R;
  control_mode = MODE_ABC;
}

void AutoBalancer::stopABCparam()
{
  std::cerr << "[AutoBalancer] stop auto balancer mode" << std::endl;
  //Guard guard(m_mutex);
  mid_coords(fix_leg_coords, 0.5,
             coordinates(ikp[":rleg"].target_p0, ikp[":rleg"].target_r0),
             coordinates(ikp[":lleg"].target_p0, ikp[":lleg"].target_r0));
  transition_count = MAX_TRANSITION_COUNT; // when start impedance, count up to 0
  transition_joint_q.resize(m_robot->numJoints());
  for (int i = 0; i < m_robot->numJoints(); i++ ) {
    transition_joint_q[i] = m_robot->joint(i)->q;
  }
  prefzmp = refzmp;
  control_mode = MODE_SYNC;
  gg_ending = gg_solved = gg_is_walking = false;
}

void AutoBalancer::startWalking ()
{
  if ( control_mode != MODE_ABC ) {
    return_control_mode = control_mode;
    OpenHRP::AutoBalancerService::AutoBalancerLimbParamSequence alps;
    alps.length(2);
    alps[0].name = ":rleg";
    alps[1].name = ":lleg";
    startABCparam(alps);
    waitABCTransition();
  }
  hrp::Vector3 cog(m_robot->calcCM());
  std::string init_support_leg (gg->get_footstep_front_leg() == ":rleg" ? ":lleg" : ":rleg");
  std::string init_swing_leg (gg->get_footstep_front_leg());
  coordinates spc, swc;
  gg->set_default_zmp_offsets(default_zmp_offsets);
  gg->initialize_gait_parameter(cog,
                                coordinates(ikp[init_support_leg].target_p0, ikp[init_support_leg].target_r0),
                                coordinates(ikp[init_swing_leg].target_p0, ikp[init_swing_leg].target_r0));
  while ( !gg->proc_one_tick(rats::gait_generator::CYCLOID) );
  gg_is_walking = gg_solved = true;
  gg_ending = false;
}

void AutoBalancer::stopWalking ()
{
  if (!gg_ending){
    gg_ending = true; // tmpolary
    /* sync */
  } else {
    /* overwrite sequencer's angle-vector when finishing steps */
    fixLegToCoords(":both", fix_leg_coords);
    gg->clear_footstep_node_list();
    if (return_control_mode == MODE_IDLE) stopABCparam();
    gg_is_walking = false;
    gg_ending = false;
  }
}

bool AutoBalancer::startABC (const OpenHRP::AutoBalancerService::AutoBalancerLimbParamSequence& alp)
{
  if (control_mode == MODE_IDLE) {
    startABCparam(alp);
    waitABCTransition();
    return_control_mode = MODE_ABC;
    return true;
  } else {
    return false;
  }
}

bool AutoBalancer::stopABC ()
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
  while (transition_count != 0) usleep(10);
  usleep(10);
}
bool AutoBalancer::goPos(const double& x, const double& y, const double& th)
{
  coordinates foot_midcoords;
  mid_coords(foot_midcoords, 0.5,
             coordinates(ikp[":rleg"].target_p0, ikp[":rleg"].target_r0),
             coordinates(ikp[":lleg"].target_p0, ikp[":lleg"].target_r0));
  gg->go_pos_param_2_footstep_list(x, y, th, foot_midcoords);
  gg->print_footstep_list();
  startWalking();
  return 0;
}

bool AutoBalancer::goVelocity(const double& vx, const double& vy, const double& vth)
{
  if (gg_is_walking && gg_solved) {
    gg->set_velocity_param(vx, vy, vth);
  } else {
    coordinates foot_midcoords;
    mid_coords(foot_midcoords, 0.5,
               coordinates(ikp[":rleg"].target_p0, ikp[":rleg"].target_r0),
               coordinates(ikp[":lleg"].target_p0, ikp[":lleg"].target_r0));
    gg->initialize_velocity_mode(foot_midcoords, vx, vy, vth);
    startWalking();
  }
  return 0;
}

bool AutoBalancer::goStop ()
{
  gg->finalize_velocity_mode();
  waitFootSteps();
  return true;
}

bool AutoBalancer::setFootSteps(const OpenHRP::AutoBalancerService::FootstepSequence& fs)
{
    coordinates tmpfs;
    std::cerr << "set_foot_steps" << std::endl;

    gg->clear_footstep_node_list();
    for (size_t i = 0; i < fs.length(); i++) {
      std::string leg(fs[i].leg);
      if (leg == ":rleg" || leg == ":lleg") {
        memcpy(tmpfs.pos.data(), fs[i].pos, sizeof(double)*3);
        tmpfs.rot = (Eigen::Quaternion<double>(fs[i].rot[0], fs[i].rot[1], fs[i].rot[2], fs[i].rot[3])).normalized().toRotationMatrix(); // rtc: (x, y, z, w) but eigen: (w, x, y, z)
        gg->append_footstep_node(leg, tmpfs);
      } else {
        std::cerr << "no such target : " << leg << std::endl;
        return false;
      }
    }
    std::cerr << "[AutoBalancer] : print footsteps " << std::endl;
    gg->append_finalize_footstep();
    gg->print_footstep_list();
    startWalking();
 return true;
}

void AutoBalancer::waitFootSteps()
{
  //while (gg_is_walking) usleep(10);
  while (gg_is_walking || transition_count != 0 )
    usleep(10);
  usleep(10);
  gg->set_offset_velocity_param(0,0,0);
}

bool AutoBalancer::setGaitGeneratorParam(const OpenHRP::AutoBalancerService::GaitGeneratorParam& i_param)
{
  gg->set_stride_parameters(i_param.stride_parameter[0], i_param.stride_parameter[1], i_param.stride_parameter[2]);
  gg->set_default_step_time(i_param.default_step_time);
  gg->set_default_step_height(i_param.default_step_height);
  gg->set_default_double_support_ratio(i_param.default_double_support_ratio);
  return true;
};

bool AutoBalancer::getGaitGeneratorParam(OpenHRP::AutoBalancerService::GaitGeneratorParam& i_param)
{
  gg->get_stride_parameters(i_param.stride_parameter[0], i_param.stride_parameter[1], i_param.stride_parameter[2]);
  i_param.default_step_time = gg->get_default_step_time();
  i_param.default_step_height = gg->get_default_step_height();
  i_param.default_double_support_ratio = gg->get_default_double_support_ratio();
  return true;
};

bool AutoBalancer::setAutoBalancerParam(const OpenHRP::AutoBalancerService::AutoBalancerParam& i_param)
{
  move_base_gain = i_param.move_base_gain;
  for (size_t i = 0; i < 2; i++)
    for (size_t j = 0; j < 3; j++)
      default_zmp_offsets[i](j) = i_param.default_zmp_offsets[i][j];
  std::cerr << "move_base_gain: " << move_base_gain << std::endl;
  std::cerr << "default_zmp_offsets: "
            << default_zmp_offsets[0](0) << " " << default_zmp_offsets[0](1) << " " << default_zmp_offsets[0](2) << " "
            << default_zmp_offsets[1](0) << " " << default_zmp_offsets[1](1) << " " << default_zmp_offsets[1](2) << std::endl;
  return true;
};

bool AutoBalancer::getAutoBalancerParam(OpenHRP::AutoBalancerService::AutoBalancerParam& i_param)
{
  i_param.move_base_gain = move_base_gain;
  for (size_t i = 0; i < 2; i++)
    for (size_t j = 0; j < 3; j++)
      i_param.default_zmp_offsets[i][j] = default_zmp_offsets[i](j);
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
  copyRatscoords2Footstep(i_param.rleg_coords, coordinates(ikp[":rleg"].current_p0, ikp[":rleg"].current_r0));
  copyRatscoords2Footstep(i_param.lleg_coords, coordinates(ikp[":lleg"].current_p0, ikp[":lleg"].current_r0));
  copyRatscoords2Footstep(i_param.support_leg_coords, gg->get_support_leg_coords());
  copyRatscoords2Footstep(i_param.swing_leg_coords, gg->get_swing_leg_coords());
  copyRatscoords2Footstep(i_param.swing_leg_src_coords, gg->get_swing_leg_src_coords());
  copyRatscoords2Footstep(i_param.swing_leg_dst_coords, gg->get_swing_leg_dst_coords());
  copyRatscoords2Footstep(i_param.dst_foot_midcoords, gg->get_dst_foot_midcoords());
  if (gg->get_support_leg() == ":rleg") {
    i_param.support_leg = OpenHRP::AutoBalancerService::RLEG;
  } else {
    i_param.support_leg = OpenHRP::AutoBalancerService::LLEG;
  }
  switch ( gg->get_current_support_state() ) {
  case 0: i_param.support_leg_with_both = OpenHRP::AutoBalancerService::BOTH; break;
  case 1: i_param.support_leg_with_both = OpenHRP::AutoBalancerService::RLEG; break;
  case 2: i_param.support_leg_with_both = OpenHRP::AutoBalancerService::LLEG; break;
  default: break;
  }
  return true;
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
  double gravity = 9.8; // [m/s^2]
  hrp::Vector3 denom, nume;
  /* sb_point[m] = nume[kg * m/s^2 * m] / denom[kg * m/s^2] */
  double mass = m_robot->totalMass();
  for (size_t j = 0; j < 2; j++) {
    nume(j) = mass * gravity * tmpcog(j);
    denom(j) = mass * gravity;
    for (size_t i = 0; i < m_ref_forceIn.size(); i++) {
      std::string sensor_name = m_ref_forceIn[i]->name();
      if ( sensor_name.find("hsensor") != std::string::npos ) { // tempolary to get arm force coords
        hrp::Sensor *sensor = m_robot->sensor(hrp::Sensor::FORCE, i);
        hrp::Vector3 fpos = sensor->link->p + (sensor->link->R) * sensor->localPos;
        nume(j) += ( (fpos(2) - ref_com_height) * tmp_forces[i](j) - fpos(j) * tmp_forces[i](2) );
        denom(j) -= tmp_forces[i](2);
      }
    }
    sb_point(j) = nume(j) / denom(j);
  }
  sb_point(2) = ref_com_height;
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


