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

#ifdef __QNX__
using std::exp;
using std::fabs;
#else
using std::isnan;
using std::isinf;
#endif

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
        std::cerr << "failed to load model[" << prop["model"] << "]" 
                  << std::endl;
        return RTC::RTC_ERROR;
    }

    // allocate memory for outPorts
    m_qRef.data.length(m_robot->numJoints());
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
    if (leg_offset_str.size() > 0 && stride_param_str.size() > 0) {
      gg = ggPtr(new rats::gait_generator(m_dt, leg_pos, stride_param(0)/*[m]*/, stride_param(1)/*[m]*/, stride_param(2)/*[deg]*/));
      gg_is_walking = gg_ending = gg_solved = false;
      if (default_zmp_offsets.size() == 0) {
        for (size_t i = 0; i < 2; i++) default_zmp_offsets.push_back(hrp::Vector3::Zero());
      }
      gg->set_default_zmp_offsets(default_zmp_offsets);
    }
    fix_leg_coords = coordinates();

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
    Guard guard(m_mutex);
    robotstateOrg2qRef();
    if (control_mode == MODE_ABC ) solveLimbIK();
    if ( m_q.data.length() != 0 ) { // initialized
      for ( int i = 0; i < m_robot->numJoints(); i++ ){
        m_q.data[i] = m_robot->joint(i)->q;
      }
      m_qOut.write();
      if ( DEBUGP ) {
        std::cerr << "q     : ";
        for ( int i = 0; i < m_q.data.length(); i++ ){
          std::cerr << " " << m_q.data[i];
        }
        std::cerr << std::endl;
      }
    }
    // {
    //   hrp::Vector3 refzmp;
    //   if (gg_is_walking) {
    //     refzmp = gg->get_refzmp();
    //     m_zmpRef.data.x = refzmp(0);
    //     m_zmpRef.data.y = refzmp(1);
    //     m_zmpRef.data.z = refzmp(2);
    //   } else {
    //     refzmp = ((m_robot->link("L_ANKLE_R"))->p+(m_robot->link("R_ANKLE_R"))->p)/2;
    //     m_zmpRef.data.x = refzmp(0);
    //     m_zmpRef.data.y = refzmp(1);
    //     m_zmpRef.data.z = refzmp(2);
    //   }
    //   m_zmpRefOut.write();
    // }
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
  if ( ikp.size() > 0 ) {
    coordinates tmp_fix_coords;
    if ( gg_is_walking ) {
      //gg->set_default_zmp_offsets(tmpzo);
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
      target_com = (m_robot->link(ikp[":rleg"].target_name)->p+
                    m_robot->link(ikp[":lleg"].target_name)->p)/2.0;
    }
  }
  if ( transition_count > 0 ) {
    for ( std::map<std::string, ABCIKparam>::iterator it = ikp.begin(); it != ikp.end(); it++ ) {
      hrp::JointPathExPtr manip = it->second.manip;
      for ( int j = 0; j < manip->numJoints(); j++ ) {
        int i = manip->joint(j)->jointId; // index in robot model
        hrp::Link* joint =  m_robot->joint(i);
        // transition_smooth_gain moves from 0 to 1
        // (/ (log (/ (- 1 0.99) 0.99)) 0.5)
        double transition_smooth_gain = 1/(1+exp(-9.19*(((MAX_TRANSITION_COUNT - transition_count) / MAX_TRANSITION_COUNT) - 0.5)));
        joint->q = ( m_qRef.data[i] - transition_joint_q[i] ) * transition_smooth_gain + transition_joint_q[i];
      }
    }
    transition_count--;
    if(transition_count <= 0){ // erase impedance param
      std::cerr << "Finished cleanup" << std::endl;
      control_mode = MODE_IDLE;
    }
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
  hrp::Link* base = m_robot->link(param.base_name);
  hrp::Link* target = m_robot->link(param.target_name);
  assert(target);
  assert(base);
  param.current_p0 = target->p;
  param.current_r0 = target->R;

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
    for ( int j = 0; j < it->second.manip->numJoints(); j++ ){
      int i = it->second.manip->joint(j)->jointId;
      m_robot->joint(i)->q = qorg[i];
    }
  }
  m_robot->rootLink()->p = base_pos_org;
  m_robot->rootLink()->R = base_rot_org;
  m_robot->calcForwardKinematics();
  hrp::Vector3 current_com = m_robot->calcCM();
  hrp::Vector3 dif_com = current_com - target_com;
  dif_com(2) = m_robot->rootLink()->p(2) - target_base_pos(2);
  m_robot->rootLink()->p = m_robot->rootLink()->p + -1 * move_base_gain * transition_smooth_gain * dif_com;
  m_robot->rootLink()->R = target_base_rot;
  m_robot->calcForwardKinematics();

  for ( std::map<std::string, ABCIKparam>::iterator it = ikp.begin(); it != ikp.end(); it++ ) {
    if (!solveLimbIKforLimb(it->second, transition_smooth_gain)) break;
  }
  if ( transition_count < 0 ) {
    transition_count++;
  }
  //if (gg_is_walking && !gg_solved) stopWalking ();
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

  for (size_t i = 0; i < alp.length(); i++) {
    const OpenHRP::AutoBalancerService::AutoBalancerLimbParam& tmpalp = alp[i];
    ABCIKparam tmp;
    tmp.base_name = std::string(tmpalp.base_name);
    tmp.target_name = std::string(tmpalp.target_name);
    tmp.manip = hrp::JointPathExPtr(new hrp::JointPathEx(m_robot, m_robot->link(tmp.base_name), m_robot->link(tmp.target_name)));
    memcpy(tmp.target2foot_offset_pos.data(), tmpalp.target2foot_offset_pos, sizeof(double)*3);
    tmp.target2foot_offset_rot = (Eigen::Quaternion<double>(tmpalp.target2foot_offset_rot[0],
                                                            tmpalp.target2foot_offset_rot[1],
                                                            tmpalp.target2foot_offset_rot[2],
                                                            tmpalp.target2foot_offset_rot[3])).normalized().toRotationMatrix();
    ikp.insert(std::pair<std::string, ABCIKparam>(std::string(tmpalp.name), tmp));
    std::cerr << "abc limb [" << std::string(tmpalp.name) << "]" << std::endl;
    std::cerr << "     base_name : " << std::string(tmpalp.base_name) << ", target_name :" << std::string(tmpalp.target_name) << std::endl;
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
  Guard guard(m_mutex);
  mid_coords(fix_leg_coords, 0.5,
             coordinates(ikp[":rleg"].target_p0, ikp[":rleg"].target_r0),
             coordinates(ikp[":lleg"].target_p0, ikp[":lleg"].target_r0));
  transition_count = MAX_TRANSITION_COUNT; // when start impedance, count up to 0
  transition_joint_q.resize(m_robot->numJoints());
  for (int i = 0; i < m_robot->numJoints(); i++ ) {
    transition_joint_q[i] = m_robot->joint(i)->q;
  }
  control_mode = MODE_SYNC;
  gg_ending = gg_solved = gg_is_walking = false;
}

void AutoBalancer::startWalking ()
{
  hrp::Vector3 cog(m_robot->calcCM());
  std::string init_support_leg (gg->get_footstep_front_leg() == ":rleg" ? ":lleg" : ":rleg");
  std::string init_swing_leg (gg->get_footstep_front_leg());
  coordinates spc, swc;
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
    //gg_ending = true; // tmpolary
    /* sync */
  } else {
    /* overwrite sequencer's angle-vector when finishing steps */
    gg_is_walking = false;
    //rb->get_foot_midcoords(fix_leg_coords);
    gg->clear_footstep_node_list();
    // coordinates rc, lc;
    // rb->get_end_coords(rc, ":rleg");
    // rb->get_end_coords(lc, ":lleg");
    // rc.translate(default_zmp_offsets[0]); /* :rleg */
    // lc.translate(default_zmp_offsets[1]); /* :lleg */
    // target_cog = (rc.pos + lc.pos) / 2.0;
    /* sync */
    gg_ending = false;
  }
}

bool AutoBalancer::startABC (const OpenHRP::AutoBalancerService::AutoBalancerLimbParamSequence& alp)
{
  if (control_mode == MODE_IDLE) {
    startABCparam(alp);
    waitABCTransition();
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
  if (control_mode == MODE_ABC ) {
    coordinates foot_midcoords;
    mid_coords(foot_midcoords, 0.5,
               coordinates(ikp[":rleg"].target_p0, ikp[":rleg"].target_r0),
               coordinates(ikp[":lleg"].target_p0, ikp[":lleg"].target_r0));
    gg->go_pos_param_2_footstep_list(x, y, th, foot_midcoords);
    gg->print_footstep_list();
    startWalking();
  }
  return 0;
}

bool AutoBalancer::goVelocity(const double& vx, const double& vy, const double& vth)
{
  if (control_mode == MODE_ABC ) {
    //    if (gg_is_walking) {
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
  if (control_mode == MODE_ABC ) {
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
  }
 return true;
}

void AutoBalancer::waitFootSteps()
{
  //while (gg_is_walking) usleep(10);
  while (gg_is_walking && gg_solved) usleep(10);
  usleep(10);
  gg->set_offset_velocity_param(0,0,0);
}

bool AutoBalancer::setGaitGeneratorParam(const OpenHRP::AutoBalancerService::GaitGeneratorParam& i_param)
{
  gg->set_stride_parameters(i_param.stride_parameter[0], i_param.stride_parameter[1], i_param.stride_parameter[2]);
  gg->set_default_step_time(i_param.default_step_time);
  gg->set_default_step_height(i_param.default_step_height);
  return true;
};

bool AutoBalancer::getGaitGeneratorParam(OpenHRP::AutoBalancerService::GaitGeneratorParam& i_param)
{
  gg->get_stride_parameters(i_param.stride_parameter[0], i_param.stride_parameter[1], i_param.stride_parameter[2]);
  i_param.default_step_time = gg->get_default_step_time();
  i_param.default_step_height = gg->get_default_step_height();
  return true;
};

bool AutoBalancer::setAutoBalancerParam(const OpenHRP::AutoBalancerService::AutoBalancerParam& i_param)
{
  move_base_gain = i_param.move_base_gain;
  for (size_t i = 0; i < 2; i++)
    for (size_t j = 0; j < 3; j++)
      default_zmp_offsets[i](j) = i_param.default_zmp_offsets[i][j];
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


