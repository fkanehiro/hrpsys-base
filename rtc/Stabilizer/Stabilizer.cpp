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
#include "util/VectorConvert.h"
#include <math.h>

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

#define MAX_TRANSITION_COUNT (2/dt)
//#define USE_EEFM_STABILIZER
static double vlimit(double value, double llimit_value, double ulimit_value);
static double switching_inpact_absorber(double force, double lower_th, double upper_th);

Stabilizer::Stabilizer(RTC::Manager* manager)
  : RTC::DataFlowComponentBase(manager),
    // <rtc-template block="initializer">
    m_qCurrentIn("qCurrent", m_qCurrent),
    m_qRefIn("qRef", m_qRef),
    m_rpyIn("rpy", m_rpy),
    m_forceLIn("forceL", m_force[1]),
    m_forceRIn("forceR", m_force[0]),
    m_zmpRefIn("zmpRef", m_zmpRef),
    m_StabilizerServicePort("StabilizerService"),
    m_basePosIn("basePosIn", m_basePos),
    m_baseRpyIn("baseRpyIn", m_baseRpy),
    m_contactStatesIn("contactStates", m_contactStates),
    m_qRefOut("q", m_qRef),
    m_tauOut("tau", m_tau),
    m_zmpOut("zmp", m_zmp),
    // for debug output
    m_originRefZmpOut("originRefZmp", m_originRefZmp),
    m_originRefCogOut("originRefCog", m_originRefCog),
    m_originRefCogVelOut("originRefCogVel", m_originRefCogVel),
    m_originNewZmpOut("originNewZmp", m_originNewZmp),
    m_originActZmpOut("originActZmp", m_originActZmp),
    m_originActCogOut("originActCog", m_originActCog),
    m_originActCogVelOut("originActCogVel", m_originActCogVel),
    m_refWrenchROut("refWrenchR", m_refWrenchR),
    m_refWrenchLOut("refWrenchL", m_refWrenchL),
    m_footCompROut("footCompR", m_footCompR),
    m_footCompLOut("footCompL", m_footCompL),
    control_mode(MODE_IDLE),
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
  std::cerr << m_profile.instance_name << ": onInitialize()" << std::endl;
  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  bindParameter("debugLevel", m_debugLevel, "0");
  
  // </rtc-template>

  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("qCurrent", m_qCurrentIn);
  addInPort("qRef", m_qRefIn);
  addInPort("forceR", m_forceRIn);
  addInPort("forceL", m_forceLIn);
  addInPort("rpy", m_rpyIn);
  addInPort("zmpRef", m_zmpRefIn);
  addInPort("basePosIn", m_basePosIn);
  addInPort("baseRpyIn", m_baseRpyIn);
  addInPort("contactStates", m_contactStatesIn);

  // Set OutPort buffer
  addOutPort("q", m_qRefOut);
  addOutPort("tau", m_tauOut);
  addOutPort("zmp", m_zmpOut);
  // for debug output
  addOutPort("originRefZmp", m_originRefZmpOut);
  addOutPort("originRefCog", m_originRefCogOut);
  addOutPort("originRefCogVel", m_originRefCogVelOut);
  addOutPort("originNewZmp", m_originNewZmpOut);
  addOutPort("originActZmp", m_originActZmpOut);
  addOutPort("originActCog", m_originActCogOut);
  addOutPort("originActCogVel", m_originActCogVelOut);
  addOutPort("refWrenchR", m_refWrenchROut);
  addOutPort("refWrenchL", m_refWrenchLOut);
  addOutPort("footCompR", m_footCompROut);
  addOutPort("footCompL", m_footCompLOut);
  
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
    std::cerr << "failed to load model[" << prop["model"] << "] in "
              << m_profile.instance_name << std::endl;
    return RTC::RTC_ERROR;
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
      ee_trans eet;
      for (size_t j = 0; j < 3; j++) {
        coil::stringTo(eet.localp(j), end_effectors_str[i*prop_num+3+j].c_str());
      }
      double tmpv[4];
      for (int j = 0; j < 4; j++ ) {
        coil::stringTo(tmpv[j], end_effectors_str[i*prop_num+6+j].c_str());
      }
      eet.localR = Eigen::AngleAxis<double>(tmpv[3], hrp::Vector3(tmpv[0], tmpv[1], tmpv[2])).toRotationMatrix(); // rotation in VRML is represented by axis + angle
      // manip2[i] = hrp::JointPathExPtr(new hrp::JointPathEx(m_robot, m_robot->link(ee_base),
      //                                                      m_robot->link(ee_target)));
      //ee_map.insert(std::pair<std::string, ee_trans>(ee_name , eet));
      ee_map.insert(std::pair<std::string, ee_trans>(ee_target , eet));
      contact_states_index_map.insert(std::pair<std::string, size_t>(ee_name, i));
    }
    m_contactStates.data.length(num);
  }

  // parameters for TPCC
  act_zmp = hrp::Vector3::Zero();
  for (int i = 0; i < ST_NUM_LEGS; i++) {
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
  }
  eefm_rot_damping_gain = 20*5;
  eefm_rot_time_const = 1;
  eefm_pos_damping_gain = 3500;
  eefm_pos_time_const = 1;
  eefm_zmp_delay_time_const[0] = eefm_zmp_delay_time_const[1] = 0.04;

  // parameters for RUNST
  double ke = 0, tc = 0;
  for (int i = 0; i < ST_NUM_LEGS; i++) {
    m_tau_x[i].setup(ke, tc, dt);
    m_tau_y[i].setup(ke, tc, dt);
    m_f_z.setup(ke, tc, dt);
  }
  pangx_ref = pangy_ref = pangx = pangy = 0;
  rdx = rdy = rx = ry = 0;
  pdr = hrp::Vector3::Zero();
  prev_act_force_z[0] = prev_act_force_z[1] = 0.0;

  sensor_names.push_back("rfsensor");
  sensor_names.push_back("lfsensor");

  is_legged_robot = false;
  for (size_t i = 0; i < 2; i++) {
    if ( m_robot->sensor<hrp::ForceSensor>(sensor_names[i]) != NULL) {
      manip2[i] = hrp::JointPathExPtr(new hrp::JointPathEx(m_robot, m_robot->rootLink(), m_robot->sensor<hrp::ForceSensor>(sensor_names[i])->link));
      is_legged_robot = true;
    }
  }

  m_qCurrent.data.length(m_robot->numJoints());
  m_qRef.data.length(m_robot->numJoints());
  m_tau.data.length(m_robot->numJoints());
  transition_joint_q.resize(m_robot->numJoints());
  qorg.resize(m_robot->numJoints());
  qrefv.resize(m_robot->numJoints());
  transition_count = 0;
  loop = 0;
  if (is_legged_robot) {
    zmp_origin_off = ee_map[m_robot->sensor<hrp::ForceSensor>(sensor_names[0])->link->name].localp(2);
  }
  total_mass = m_robot->totalMass();
  ref_zmp_aux = hrp::Vector3::Zero();
  for (size_t i = 0; i < m_contactStates.data.length(); i++) {
    contact_states.push_back(true);
    prev_contact_states.push_back(true);
  }

  // for debug output
  m_originRefZmp.data.x = m_originRefZmp.data.y = m_originRefZmp.data.z = 0.0;
  m_originRefCog.data.x = m_originRefCog.data.y = m_originRefCog.data.z = 0.0;
  m_originRefCogVel.data.x = m_originRefCogVel.data.y = m_originRefCogVel.data.z = 0.0;
  m_originNewZmp.data.x = m_originNewZmp.data.y = m_originNewZmp.data.z = 0.0;
  m_originActZmp.data.x = m_originActZmp.data.y = m_originActZmp.data.z = 0.0;
  m_originActCog.data.x = m_originActCog.data.y = m_originActCog.data.z = 0.0;
  m_originActCogVel.data.x = m_originActCogVel.data.y = m_originActCogVel.data.z = 0.0;
  m_refWrenchR.data.length(6); m_refWrenchL.data.length(6);
  m_refWrenchR.data[0] = m_refWrenchR.data[1] = m_refWrenchR.data[2] = m_refWrenchR.data[3] = m_refWrenchR.data[4] = m_refWrenchR.data[5] = 0.0;
  m_refWrenchL.data[0] = m_refWrenchL.data[1] = m_refWrenchL.data[2] = m_refWrenchL.data[3] = m_refWrenchL.data[4] = m_refWrenchL.data[5] = 0.0;
  m_footCompR.data.length(6); m_footCompL.data.length(6);
  m_footCompR.data[0] = m_footCompR.data[1] = m_footCompR.data[2] = m_footCompR.data[3] = m_footCompR.data[4] = m_footCompR.data[5] = 0.0;
  m_footCompL.data[0] = m_footCompL.data[1] = m_footCompL.data[2] = m_footCompL.data[3] = m_footCompL.data[4] = m_footCompL.data[5] = 0.0;

  return RTC::RTC_OK;
}


/*
RTC::ReturnCode_t Stabilizer::onFinalize()
{
  return RTC::RTC_OK;
}
*/

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
  std::cout << m_profile.instance_name<< ": onActivated(" << ec_id << ")" << std::endl;
  return RTC::RTC_OK;
}

RTC::ReturnCode_t Stabilizer::onDeactivated(RTC::UniqueId ec_id)
{
  std::cout << m_profile.instance_name<< ": onDeactivated(" << ec_id << ")" << std::endl;
  return RTC::RTC_OK;
}

bool Stabilizer::calcZMP(hrp::Vector3& ret_zmp, const double zmp_z)
{
  double tmpzmpx = 0;
  double tmpzmpy = 0;
  double tmpfz = 0, tmpfz2 = 0.0;
  for (size_t i = 0; i < 2; i++) {
    hrp::ForceSensor* sensor = m_robot->sensor<hrp::ForceSensor>(sensor_names[i]);
    hrp::Vector3 fsp = sensor->link->p + sensor->link->R * sensor->localPos;
    hrp::Matrix33 tmpR;
    rats::rotm3times(tmpR, sensor->link->R, sensor->localR);
    hrp::Vector3 nf = tmpR * hrp::Vector3(m_force[i].data[0], m_force[i].data[1], m_force[i].data[2]);
    hrp::Vector3 nm = tmpR * hrp::Vector3(m_force[i].data[3], m_force[i].data[4], m_force[i].data[5]);
    tmpzmpx += nf(2) * fsp(0) - (fsp(2) - zmp_z) * nf(0) - nm(1);
    tmpzmpy += nf(2) * fsp(1) - (fsp(2) - zmp_z) * nf(1) + nm(0);
    tmpfz += nf(2);
    prev_act_force_z[i] = 0.85 * prev_act_force_z[i] + 0.15 * nf(2); // filter, cut off 5[Hz]
  }
  tmpfz2 = prev_act_force_z[0] + prev_act_force_z[1];
  if (tmpfz2 < 50) {
    ret_zmp = act_zmp;
    return false; // in the air
  } else {
    ret_zmp = hrp::Vector3(tmpzmpx / tmpfz, tmpzmpy / tmpfz, zmp_z);
    return true; // on ground
  }
};

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
  if (m_forceRIn.isNew()) {
    m_forceRIn.read();
  }
  if (m_forceLIn.isNew()) {
    m_forceLIn.read();
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
      contact_states[i] = m_contactStates.data[i];
    }
  }

  if (is_legged_robot) {
    getCurrentParameters();
    getTargetParameters();
    getActualParameters();
    switch (control_mode) {
    case MODE_IDLE:
      // if (DEBUGP2) std::cerr << "IDLE"<< std::endl;
      break;
    case MODE_AIR:
      // if (DEBUGP2) std::cerr << "AIR"<< std::endl;
      if ( transition_count == 0 && on_ground ) sync_2_st();
      break;
    case MODE_ST:
      // if (DEBUGP2) std::cerr << "ST"<< std::endl;
      //calcRUNST();
#ifdef USE_EEFM_STABILIZER
      calcEEForceMomentControl();
#else
      calcTPCC();
#endif
      if ( transition_count == 0 && !on_ground ) control_mode = MODE_SYNC_TO_AIR;
      break;
    case MODE_SYNC_TO_IDLE:
      // std::cerr << "SYNCIDLE"<< std::endl;
      sync_2_idle();
      control_mode = MODE_IDLE;
      break;
    case MODE_SYNC_TO_AIR:
      // std::cerr << "SYNCAIR"<< std::endl;
      sync_2_idle();
      control_mode = MODE_AIR;
      break;
    }
  }
  //calcTorque ();
  if ( m_robot->numJoints() == m_qRef.data.length() ) {
    if (is_legged_robot) {
      for ( int i = 0; i < m_robot->numJoints(); i++ ){
        m_qRef.data[i] = m_robot->joint(i)->q;
        //m_tau.data[i] = m_robot->joint(i)->u;
      }
      m_zmp.data.x = rel_act_zmp(0);
      m_zmp.data.y = rel_act_zmp(1);
      m_zmp.data.z = rel_act_zmp(2);
      m_zmpOut.write();
      //m_tauOut.write();
      // for debug output
      m_originRefZmp.data.x = ref_zmp(0); m_originRefZmp.data.y = ref_zmp(1); m_originRefZmp.data.z = ref_zmp(2);
      m_originRefCog.data.x = ref_cog(0); m_originRefCog.data.y = ref_cog(1); m_originRefCog.data.z = ref_cog(2);
      m_originRefCogVel.data.x = ref_cogvel(0); m_originRefCogVel.data.y = ref_cogvel(1); m_originRefCogVel.data.z = ref_cogvel(2);
      m_originNewZmp.data.x = new_refzmp(0); m_originNewZmp.data.y = new_refzmp(1); m_originNewZmp.data.z = new_refzmp(2);
      m_originActZmp.data.x = act_zmp(0); m_originActZmp.data.y = act_zmp(1); m_originActZmp.data.z = act_zmp(2);
      m_originActCog.data.x = act_cog(0); m_originActCog.data.y = act_cog(1); m_originActCog.data.z = act_cog(2);
      m_originActCogVel.data.x = act_cogvel(0); m_originActCogVel.data.y = act_cogvel(1); m_originActCogVel.data.z = act_cogvel(2);
      m_refWrenchR.data[0] = ref_foot_force[0](0); m_refWrenchR.data[1] = ref_foot_force[0](1); m_refWrenchR.data[2] = ref_foot_force[0](2);
      m_refWrenchR.data[3] = ref_foot_moment[0](0); m_refWrenchR.data[4] = ref_foot_moment[0](1); m_refWrenchR.data[5] = ref_foot_moment[0](2);
      m_refWrenchL.data[0] = ref_foot_force[1](0); m_refWrenchL.data[1] = ref_foot_force[1](1); m_refWrenchL.data[2] = ref_foot_force[1](2);
      m_refWrenchL.data[3] = ref_foot_moment[1](0); m_refWrenchL.data[4] = ref_foot_moment[1](1); m_refWrenchL.data[5] = ref_foot_moment[1](2);
      m_footCompR.data[2] = f_zctrl[0]; m_footCompL.data[2] = f_zctrl[1];
      m_footCompR.data[3] = d_foot_rpy[0](0); m_footCompR.data[4] = d_foot_rpy[0](1);
      m_footCompL.data[3] = d_foot_rpy[1](0); m_footCompL.data[4] = d_foot_rpy[1](1);
      m_originRefZmpOut.write();
      m_originRefCogOut.write();
      m_originRefCogVelOut.write();
      m_originNewZmpOut.write();
      m_originActZmpOut.write();
      m_originActCogOut.write();
      m_originActCogVelOut.write();
      m_refWrenchROut.write(); m_refWrenchLOut.write();
      m_footCompROut.write(); m_footCompLOut.write();
    }
    m_qRefOut.write();
  }

  return RTC::RTC_OK;
}

void Stabilizer::calcContactMatrix (hrp::dmatrix& tm, const std::vector<hrp::Vector3>& contact_p)
{
  tm.resize(6,6*contact_p.size());
  tm.setZero();
  for (size_t c = 0; c < contact_p.size(); c++) {
    for (size_t i = 0; i < 6; i++) tm(i,(c*6)+i) = 1.0;
    hrp::Matrix33 cm;
    rats::outer_product_matrix(cm, contact_p[c]);
    for (size_t i = 0; i < 3; i++)
      for (size_t j = 0; j < 3; j++) tm(i+3,(c*6)+j) = cm(i,j);
  }
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
  for (size_t j = 0; j < 2; j++) contact_p.push_back(m_robot->sensor<hrp::ForceSensor>(sensor_names[j])->link->p);
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
    hrp::JointPathEx jm = hrp::JointPathEx(m_robot, m_robot->rootLink(), m_robot->sensor<hrp::ForceSensor>(sensor_names[j])->link);
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

void Stabilizer::getCurrentParameters ()
{
  current_root_p = m_robot->rootLink()->p;
  current_root_R = m_robot->rootLink()->R;
  for ( int i = 0; i < m_robot->numJoints(); i++ ){
    qorg[i] = m_robot->joint(i)->q;
  }
}

hrp::Matrix33 Stabilizer::OrientRotationMatrix (const hrp::Matrix33& rot, const hrp::Vector3& axis1, const hrp::Vector3& axis2)
{
  hrp::Vector3 vv = axis1.cross(axis2);
  if (fabs(vv.norm()-0.0) < 1e-5) {
    return rot;
  } else {
    Eigen::AngleAxis<double> tmpr(std::asin(vv.norm()), vv.normalized());
    return tmpr.toRotationMatrix() * rot;
  }
}

void Stabilizer::calcFootOriginCoords (hrp::Vector3& foot_origin_pos, hrp::Matrix33& foot_origin_rot)
{
  rats::coordinates leg_c[2], tmpc;
  hrp::Vector3 ez = hrp::Vector3::UnitZ();
  hrp::Vector3 ex = hrp::Vector3::UnitX();
  for (size_t i = 0; i < 2; i++) {
    hrp::Link* target = m_robot->sensor<hrp::ForceSensor>(sensor_names[i])->link;
    leg_c[i].pos = target->p;
    leg_c[i].rot = OrientRotationMatrix(target->R, (target->R * ez), ez);
    hrp::Vector3 xv1 = target->R * ex;
    xv1(2)=0.0;
    xv1.normalize();
    hrp::Vector3 xv2 = leg_c[i].rot * ex;
    xv2(2)=0.0;
    xv2.normalize();
    leg_c[i].rot = OrientRotationMatrix(leg_c[i].rot, xv1, xv2);
  }
  if (contact_states[contact_states_index_map["rleg"]] &&
      contact_states[contact_states_index_map["lleg"]]) {
    rats::mid_coords(tmpc, 0.5, leg_c[0], leg_c[1]);
    foot_origin_pos = tmpc.pos;
    foot_origin_rot = tmpc.rot;
  } else if (contact_states[contact_states_index_map["rleg"]]) {
    foot_origin_pos = leg_c[0].pos;
    foot_origin_rot = leg_c[0].rot;
  } else {
    foot_origin_pos = leg_c[1].pos;
    foot_origin_rot = leg_c[1].rot;
  }
}

void Stabilizer::getActualParameters ()
{
#ifdef USE_EEFM_STABILIZER
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
  hrp::Vector3 foot_origin_pos;
  hrp::Matrix33 foot_origin_rot;
  calcFootOriginCoords (foot_origin_pos, foot_origin_rot);
#else
  for ( int i = 0; i < m_robot->numJoints(); i++ ) {
    m_robot->joint(i)->q = qorg[i];
  }
  m_robot->rootLink()->p = current_root_p;
  m_robot->rootLink()->R = current_root_R;
  m_robot->calcForwardKinematics();
#endif
  // cog
  act_cog = m_robot->calcCM();
  // zmp
  on_ground = false;
  if (is_legged_robot && ( m_force[0].data.length() > 0 && m_force[1].data.length() > 0 ))
#ifdef USE_EEFM_STABILIZER
    on_ground = calcZMP(act_zmp, zmp_origin_off+foot_origin_pos(2));
#else
    on_ground = calcZMP(act_zmp, ref_zmp(2));
#endif

  // convert absolute (in st) -> root-link relative
  rel_act_zmp = m_robot->rootLink()->R.transpose() * (act_zmp - m_robot->rootLink()->p);
#ifdef USE_EEFM_STABILIZER
  // world (current-tmp) => local (foot_origin)
  act_zmp = foot_origin_rot.transpose() * (act_zmp - foot_origin_pos);
  act_cog = foot_origin_rot.transpose() * (act_cog - foot_origin_pos);
  //act_cogvel = foot_origin_rot.transpose() * act_cogvel;
  if (contact_states != prev_contact_states) {
    act_cogvel = (foot_origin_rot.transpose() * prev_act_foot_origin_rot) * act_cogvel;
  } else {
    act_cogvel = (act_cog - prev_act_cog)/dt;
  }
  prev_act_foot_origin_rot = foot_origin_rot;
  //act_cogvel = 0.8 * prev_act_cogvel + 0.2 * act_cogvel;
  act_cogvel = 0.9 * prev_act_cogvel + 0.1 * act_cogvel;
  prev_act_cog = act_cog;
  prev_act_cogvel = act_cogvel;
  //act_root_rot = m_robot->rootLink()->R;

  // new ZMP calculation
  // Kajita's feedback low
  hrp::Vector3 dcog=foot_origin_rot * (ref_cog - act_cog);
  hrp::Vector3 dcogvel=foot_origin_rot * (ref_cogvel - act_cogvel);
  hrp::Vector3 dzmp=foot_origin_rot * (ref_zmp - act_zmp);
  new_refzmp = foot_origin_rot * new_refzmp + foot_origin_pos;
  for (size_t i = 0; i < 2; i++) {
    new_refzmp(i) += eefm_k1[i] * transition_smooth_gain * dcog(i) + eefm_k2[i] * transition_smooth_gain * dcogvel(i) + eefm_k3[i] * transition_smooth_gain * dzmp(i) + ref_zmp_aux(i);
  }
  if (DEBUGP) {
    std::cerr << "COG [" << ref_cog(0)*1e3 << " " << ref_cog(1)*1e3 << " " << ref_cog(2)*1e3 << "] [" << act_cog(0)*1e3 << " " << act_cog(1)*1e3 << " " << act_cog(2)*1e3 << "]" << std::endl;
    std::cerr << "vel [" << ref_cogvel(0) << " " << ref_cogvel(1) << " " << ref_cogvel(2) << "] [" << act_cogvel(0) << " " << act_cogvel(1) << " " << act_cogvel(2) << "]" << std::endl;
    std::cerr << "ZMP [" << ref_zmp(0)*1e3 << " " << ref_zmp(1)*1e3 << " " << ref_zmp(2)*1e3 << "] [" << act_zmp(0)*1e3 << " " << act_zmp(1)*1e3 << " " << act_zmp(2)*1e3 << "]" << std::endl;
    std::cerr << "dZMP [" << (new_refzmp(0)-ref_zmp(0)) *1e3 << " " << (new_refzmp(1)-ref_zmp(1))*1e3 << " " << (new_refzmp(2)-ref_zmp(2))*1e3 << "]" << std::endl;
  }

  // distribute new ZMP into foot force & moment
  double alpha;
  hrp::Vector3 tau_0 = hrp::Vector3::Zero();
  {
    double leg_margin = 0.065; // [m]
    hrp::Vector3 ee_pos[2];
    for (size_t i = 0; i < 2; i++) {
      hrp::Link* target = m_robot->sensor<hrp::ForceSensor>(sensor_names[i])->link;
      ee_pos[i] = target->p + target->R * ee_map[target->name].localp;
    }
    // tmp
    double ledge=ee_pos[1](1) - leg_margin;
    double redge=ee_pos[0](1) + leg_margin;
    if (ledge < new_refzmp(1)) {
      alpha = 0.0;
    } else if (redge > new_refzmp(1)) {
      alpha = 1.0;
    } else {
      alpha = fabs(new_refzmp(1) - ledge)/ fabs(ledge-redge);
    }
    if (DEBUGP) {
      std::cerr << "alpha [" << alpha << "]" << std::endl;
    }
    ref_foot_force[0] = hrp::Vector3(0,0, alpha * 9.8 * total_mass);
    ref_foot_force[1] = hrp::Vector3(0,0, (1-alpha) * 9.8 * total_mass);
    for (size_t i = 0; i < 2; i++) {
      tau_0 -= (ee_pos[i] - new_refzmp).cross(ref_foot_force[i]);
    }
    if ( alpha == 0.0 ) {
      ref_foot_moment[0] = hrp::Vector3::Zero();
      ref_foot_moment[1] = -1 * (ee_pos[1] - new_refzmp).cross(ref_foot_force[1]);
    } else if ( alpha == 1.0 ) {
      ref_foot_moment[1] = hrp::Vector3::Zero();
      ref_foot_moment[0] = -1 * (ee_pos[0] - new_refzmp).cross(ref_foot_force[0]);
    } else { // double support
      // employ foot distribution coords
      hrp::Vector3 foot_dist_coords_y = (ee_pos[1] - ee_pos[0]); // e_y'
      foot_dist_coords_y(2) = 0.0;
      foot_dist_coords_y.normalize();
      hrp::Vector3 foot_dist_coords_x = hrp::Vector3(foot_dist_coords_y.cross(hrp::Vector3::UnitZ())); // e_x'
      hrp::Matrix33 foot_dist_coords_rot;
      foot_dist_coords_rot(0,0) = foot_dist_coords_x(0);
      foot_dist_coords_rot(1,0) = foot_dist_coords_x(1);
      foot_dist_coords_rot(2,0) = foot_dist_coords_x(2);
      foot_dist_coords_rot(0,1) = foot_dist_coords_y(0);
      foot_dist_coords_rot(1,1) = foot_dist_coords_y(1);
      foot_dist_coords_rot(2,1) = foot_dist_coords_y(2);
      foot_dist_coords_rot(0,2) = 0;
      foot_dist_coords_rot(1,2) = 0;
      foot_dist_coords_rot(2,2) = 1;
      hrp::Vector3 tau_0_f = foot_dist_coords_rot.transpose() * tau_0; // tau_0'
      // x
      // right
      if (tau_0_f(0) > 0) ref_foot_moment[0](0) = tau_0_f(0);
      else ref_foot_moment[0](0) = 0;
      // left
      if (tau_0_f(0) > 0) ref_foot_moment[1](0) = 0;
      else ref_foot_moment[1](0) = tau_0_f(0);
      // y
      ref_foot_moment[0](1) = tau_0_f(1) * alpha;
      ref_foot_moment[1](1) = tau_0_f(1) * (1-alpha);
      ref_foot_moment[0](2) = ref_foot_moment[1](2) = 0.0;
      // foot_dist_coords local => world
      ref_foot_moment[0] = foot_dist_coords_rot * ref_foot_moment[0];
      ref_foot_moment[1] = foot_dist_coords_rot * ref_foot_moment[1];
    }
    ref_foot_moment[0] = foot_origin_rot.transpose() * ref_foot_moment[0];
    ref_foot_moment[1] = foot_origin_rot.transpose() * ref_foot_moment[1];
    if (DEBUGP) {
      std::cerr << "tau [" << tau_0(0) << " " << tau_0(1) << " " << tau_0(2) << "]" << std::endl;
      std::cerr << "fR [" << ref_foot_force[0](0) << " " << ref_foot_force[0](1) << " " << ref_foot_force[0](2) << "]" << std::endl;
      std::cerr << "fL [" << ref_foot_force[1](0) << " " << ref_foot_force[1](1) << " " << ref_foot_force[1](2) << "]" << std::endl;
      std::cerr << "tR [" << ref_foot_moment[0](0) << " " << ref_foot_moment[0](1) << " " << ref_foot_moment[0](2) << "]" << std::endl;
      std::cerr << "tL [" << ref_foot_moment[1](0) << " " << ref_foot_moment[1](1) << " " << ref_foot_moment[1](2) << "]" << std::endl;
    }
    // for debug output
    new_refzmp = foot_origin_rot.transpose() * (new_refzmp - foot_origin_pos);
  }
  //rpy control
  {
    hrp::Vector3 act_root_rpy = hrp::rpyFromRot(m_robot->rootLink()->R);
    hrp::Vector3 ref_root_rpy = hrp::rpyFromRot(target_root_R);
    for (size_t i = 0; i < 2; i++) {
      d_rpy[i] = transition_smooth_gain * (k_brot_p[i] * (ref_root_rpy(i) - act_root_rpy(i)) - 1/k_brot_tc[i] * d_rpy[i]) * dt + d_rpy[i];
    }
  }

  // foor modif
  {
    double fz_diff = 0;
    double fz[2];
    // moment control
#define deg2rad(x) ((x) * M_PI / 180.0)
    for (size_t i = 0; i < 2; i++) {
      hrp::Sensor* sensor = m_robot->sensor<hrp::ForceSensor>(sensor_names[i]);
      hrp::Vector3 sensor_force = (sensor->link->R * sensor->localR) * hrp::Vector3(m_force[i].data[0], m_force[i].data[1], m_force[i].data[2]);
      hrp::Vector3 sensor_moment = (sensor->link->R * sensor->localR) * hrp::Vector3(m_force[i].data[3], m_force[i].data[4], m_force[i].data[5]);
      hrp::Vector3 ee_moment = (sensor->link->R * (sensor->localPos - ee_map[sensor->link->name].localp)).cross(sensor_force) + sensor_moment;
      ee_moment = foot_origin_rot.transpose() * ee_moment;
      fz_diff += (i==0? -sensor_force(2) : sensor_force(2));
      fz[i] = sensor_force(2);
      // calcDampingControl
      d_foot_rpy[i](0) = calcDampingControl(ref_foot_moment[i](0), ee_moment(0), d_foot_rpy[i](0), eefm_rot_damping_gain, eefm_rot_time_const);
      d_foot_rpy[i](1) = calcDampingControl(ref_foot_moment[i](1), ee_moment(1), d_foot_rpy[i](1), eefm_rot_damping_gain, eefm_rot_time_const);
      d_foot_rpy[i](0) = vlimit(d_foot_rpy[i](0), deg2rad(-10.0), deg2rad(10.0));
      d_foot_rpy[i](1) = vlimit(d_foot_rpy[i](1), deg2rad(-10.0), deg2rad(10.0));
    }
    // fz control
    // foot force difference control version
    // if ( (contact_states[contact_states_index_map["rleg"]] &&
    //       contact_states[contact_states_index_map["lleg"]]) ||
    //      (isContact(0) && isContact(1)) ) {
    //   zctrl = calcDampingControl (ref_foot_force[1](2)-ref_foot_force[0](2),
    //                               fz_diff, zctrl, eefm_pos_damping_gain, eefm_pos_time_const);
    // } else {
    //   zctrl = calcDampingControl (0, 0, zctrl, eefm_pos_damping_gain, 0.02);
    // }
    // // zctrl = vlimit(zctrl, -0.02, 0.02);
    // zctrl = vlimit(zctrl, -0.05, 0.05);
    // f_zctrl[0] = -0.5 * zctrl;
    // f_zctrl[1] = 0.5 * zctrl;
    // foot force independent damping control
    for (size_t i = 0; i < 2; i++) {
      f_zctrl[i] = calcDampingControl (ref_foot_force[i](2),
                                       fz[i], f_zctrl[i], eefm_pos_damping_gain, eefm_pos_time_const);
      f_zctrl[i] = vlimit(f_zctrl[i], -0.05, 0.05);
    }
  }


#endif
  for ( int i = 0; i < m_robot->numJoints(); i++ ){
    m_robot->joint(i)->q = qrefv[i];
  }
  m_robot->rootLink()->p = target_root_p;
  m_robot->rootLink()->R = target_root_R;
  if ( !(control_mode == MODE_IDLE || control_mode == MODE_AIR) ) {
    for (size_t i = 0; i < 2; i++) {
      for ( int j = 0; j < manip2[i]->numJoints(); j++ ){
        int idx = manip2[i]->joint(j)->jointId;
        m_robot->joint(idx)->q = qorg[idx];
      }
    }
    m_robot->rootLink()->p(0) = current_root_p(0);
    m_robot->rootLink()->p(1) = current_root_p(1);
    m_robot->rootLink()->R = current_root_R;
    m_robot->calcForwardKinematics();
  }
  copy (contact_states.begin(), contact_states.end(), prev_contact_states.begin());
}

void Stabilizer::getTargetParameters ()
{
  // update internal robot model
  if ( transition_count == 0 ) {
    transition_smooth_gain = 1.0;
  } else {
    transition_smooth_gain = 1/(1+exp(-9.19*(((MAX_TRANSITION_COUNT - std::fabs(transition_count)) / MAX_TRANSITION_COUNT) - 0.5)));
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
  ref_zmp = hrp::Vector3(m_zmpRef.data.x, m_zmpRef.data.y, m_zmpRef.data.z);
#ifdef USE_EEFM_STABILIZER
  // apply inverse system
  hrp::Vector3 tmp_ref_zmp = ref_zmp + eefm_zmp_delay_time_const[0] * (ref_zmp - prev_ref_zmp) / dt;
  prev_ref_zmp = ref_zmp;
  ref_zmp = tmp_ref_zmp;
#endif
  ref_cog = m_robot->calcCM();

  for (size_t i = 0; i < 2; i++) {
    hrp::Sensor* sen = m_robot->sensor<hrp::ForceSensor>(sensor_names[i]);
    if ( sen != NULL) {
      target_foot_p[i] = sen->link->p + sen->link->R * ee_map[sen->link->name].localp;
      target_foot_R[i] = sen->link->R * ee_map[sen->link->name].localR;
    }
  }
#ifdef USE_EEFM_STABILIZER
  //
  hrp::Vector3 foot_origin_pos;
  hrp::Matrix33 foot_origin_rot;
  calcFootOriginCoords (foot_origin_pos, foot_origin_rot);
  // initialize for new_refzmp
  new_refzmp = ref_zmp;
  rel_cog = m_robot->rootLink()->R.transpose() * (ref_cog-m_robot->rootLink()->p);
  // world (current-tmp) => local (foot_origin)
  zmp_origin_off = ref_zmp(2) - foot_origin_pos(2);
  ref_zmp = foot_origin_rot.transpose() * (ref_zmp - foot_origin_pos);
  ref_cog = foot_origin_rot.transpose() * (ref_cog - foot_origin_pos);
  new_refzmp = foot_origin_rot.transpose() * (new_refzmp - foot_origin_pos);
  if (contact_states != prev_contact_states) {
    ref_cogvel = (foot_origin_rot.transpose() * prev_ref_foot_origin_rot) * ref_cogvel;
  } else {
    ref_cogvel = (ref_cog - prev_ref_cog)/dt;
  }
  prev_ref_foot_origin_rot = foot_origin_rot;
#else
  ref_cogvel = (ref_cog - prev_ref_cog)/dt;
#endif
  prev_ref_cog = ref_cog;
}

void Stabilizer::calcTPCC() {
  if ( m_robot->numJoints() == m_qRef.data.length() ) {

    // stabilizer loop
    if ( ( m_force[ST_LEFT].data.length() > 0 && m_force[ST_RIGHT].data.length() > 0 ) ) {
      // Choi's feedback low
      hrp::Vector3 cog = m_robot->calcCM();
      hrp::Vector3 newcog = hrp::Vector3::Zero();
#ifdef USE_EEFM_STABILIZER
      hrp::Vector3 foot_origin_pos;
      hrp::Matrix33 foot_origin_rot;
      calcFootOriginCoords(foot_origin_pos, foot_origin_rot);
      hrp::Vector3 dcog = foot_origin_rot * (ref_cog - act_cog);
      hrp::Vector3 dzmp = foot_origin_rot * (ref_zmp - act_zmp);
#else
      hrp::Vector3 dcog = (ref_cog - act_cog);
      hrp::Vector3 dzmp = (ref_zmp - act_zmp);
#endif
      for (size_t i = 0; i < 2; i++) {
        double uu = ref_cogvel(i) - k_tpcc_p[i] * transition_smooth_gain * dzmp(i)
                                  + k_tpcc_x[i] * transition_smooth_gain * dcog(i);
        newcog(i) = uu * dt + cog(i);
      }

      //rpy control
      {
        hrp::Sensor* sen = m_robot->sensor<hrp::RateGyroSensor>("gyrometer");
        if (sen != NULL) {
          hrp::Matrix33 act_Rs(hrp::rotFromRpy(m_rpy.data.r, m_rpy.data.p, m_rpy.data.y));
          hrp::Matrix33 tmpm, act_Rb;
          rats::rotm3times(tmpm, hrp::Matrix33(sen->link->R * sen->localR).transpose(), m_robot->rootLink()->R);
          rats::rotm3times(act_Rb, act_Rs, tmpm);
          hrp::Vector3 act_rpy = hrp::rpyFromRot(act_Rb);
          hrp::Vector3 ref_rpy = hrp::rpyFromRot(target_root_R);
          for (size_t i = 0; i < 2; i++) {
            d_rpy[i] = transition_smooth_gain * (k_brot_p[i] * (ref_rpy(i) - act_rpy(i)) - 1/k_brot_tc[i] * d_rpy[i]) * dt + d_rpy[i];
          }
          rats::rotm3times(current_root_R, target_root_R, hrp::rotFromRpy(d_rpy[0], d_rpy[1], 0));
          m_robot->rootLink()->R = current_root_R;
        }
      }

      // ee target => link-origin target
      for (size_t i = 0; i < 2; i++) {
        hrp::Link* target = m_robot->sensor<hrp::ForceSensor>(sensor_names[i])->link;
        hrp::Matrix33 tmpR;
        rats::rotm3times(tmpR, target_foot_R[i], ee_map[target->name].localR.transpose());
        target_foot_R[i] = tmpR;
        target_foot_p[i] -=  target_foot_R[i] * ee_map[target->name].localp;
      }
      // solveIK
      //for (size_t jj = 0; jj < 5; jj++) {
      for (size_t jj = 0; jj < 3; jj++) {
        hrp::Vector3 tmpcm = m_robot->calcCM();
        for (size_t i = 0; i < 2; i++) {
          m_robot->rootLink()->p(i) = m_robot->rootLink()->p(i) + 0.9 * (newcog(i) - tmpcm(i));
        }
        m_robot->calcForwardKinematics();
        for (size_t i = 0; i < 2; i++) {
          hrp::Link* target = m_robot->sensor<hrp::ForceSensor>(sensor_names[i])->link;
          hrp::Vector3 vel_p, vel_r;
          vel_p = target_foot_p[i] - target->p;
          rats::difference_rotation(vel_r, target->R, target_foot_R[i]);
          manip2[i]->calcInverseKinematics2Loop(vel_p, vel_r, 1.0, 0.001, 0.01, &qrefv);
        }
      }
    }
  }
}

void Stabilizer::calcEEForceMomentControl() {
  if ( m_robot->numJoints() == m_qRef.data.length() ) {

    // stabilizer loop
    if ( ( m_force[0].data.length() > 0 && m_force[1].data.length() > 0 ) ) {
      // return to referencea
      m_robot->rootLink()->R = target_root_R;
      m_robot->rootLink()->p = target_root_p;
      for ( int i = 0; i < m_robot->numJoints(); i++ ) {
        m_robot->joint(i)->q = qrefv[i];
      }

      //rpy control
      rats::rotm3times(current_root_R, target_root_R, hrp::rotFromRpy(d_rpy[0], d_rpy[1], 0));
      m_robot->rootLink()->R = current_root_R;
      m_robot->rootLink()->p = target_root_p + target_root_R * rel_cog - current_root_R * rel_cog;
      m_robot->calcForwardKinematics();

      // foor modif
      hrp::Vector3 total_target_foot_p[2];
      hrp::Matrix33 total_target_foot_R[2];
      {
        // moment control
#define deg2rad(x) ((x) * M_PI / 180.0)
        for (size_t i = 0; i < 2; i++) {
          rats::rotm3times(total_target_foot_R[i], target_foot_R[i], hrp::rotFromRpy(-d_foot_rpy[i](0), -d_foot_rpy[i](1), 0));
        }
        for (size_t i = 0; i < 2; i++) {
          hrp::Link* target = m_robot->sensor<hrp::ForceSensor>(sensor_names[i])->link;
          total_target_foot_p[i](0) = target_foot_p[i](0);
          total_target_foot_p[i](1) = target_foot_p[i](1);
          // foot force difference control version
          // total_target_foot_p[i](2) = target_foot_p[i](2) + (i==0?0.5:-0.5)*zctrl;
          // foot force independent damping control
          total_target_foot_p[i](2) = target_foot_p[i](2) - f_zctrl[i];
        }
      }

      // ee=>link-origin
      for (size_t i = 0; i < 2; i++) {
        hrp::Link* target = m_robot->sensor<hrp::ForceSensor>(sensor_names[i])->link;
        hrp::Matrix33 tmpR;
        rats::rotm3times(tmpR, total_target_foot_R[i], ee_map[target->name].localR.transpose());
        total_target_foot_R[i] = tmpR;
        total_target_foot_p[i] -= total_target_foot_R[i] * ee_map[target->name].localp;
      }
      // solveIK
      for (size_t jj = 0; jj < 3; jj++) {
        m_robot->calcForwardKinematics();
        for (size_t i = 0; i < 2; i++) {
          hrp::Link* target = m_robot->sensor<hrp::ForceSensor>(sensor_names[i])->link;
          hrp::Vector3 vel_p, vel_r;
          vel_p = total_target_foot_p[i] - target->p;
          rats::difference_rotation(vel_r, target->R, total_target_foot_R[i]);
          vel_p *= transition_smooth_gain;
          vel_r *= transition_smooth_gain;
          manip2[i]->calcInverseKinematics2Loop(vel_p, vel_r, 1.0, 0.001, 0.01, &qrefv);
        }
      }
    }
  }
}

double Stabilizer::calcDampingControl (const double tau_d, const double tau, const double prev_d,
                                       const double DD, const double TT)
{
  return (1/DD * (tau_d - tau) - 1/TT * prev_d) * dt + prev_d;
};

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
    if ( ( m_force[ST_LEFT].data.length() > 0 && m_force[ST_RIGHT].data.length() > 0 )
         //( m_force[ST_LEFT].data[2] > m_robot->totalMass()/4 || m_force[ST_RIGHT].data[2] > m_robot->totalMass()/4 )
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
      double tau_y_total = (m_force[1].data[4] + m_force[0].data[4]) / 2;
      double dpz;
      if (DEBUGP2) {
        std::cerr << "tq limit " << tq_x_ulimit << " " << tq_x_llimit << " " << tq_y_ulimit << " " << tq_y_llimit << std::endl;
      }
      for (size_t i = 0; i < 2; i++) {
        // dleg_x[i] = m_tau_x[i].update(m_force[i].data[3], tau_xl[i]);
        // dleg_y[i] = m_tau_y[i].update(m_force[i].data[4], tau_yl[i]);
        //dleg_x[i] = m_tau_x[i].update(m_force[i].data[3], tau_xl[i]);
        dleg_x[i] = m_tau_x[i].update(0,0);
        dleg_y[i] = m_tau_y[i].update(tau_y_total, tau_yl[i]);
        if (DEBUGP2) {
          std::cerr << i << " dleg_x " << dleg_x[i] << std::endl;
          std::cerr << i << " dleg_y " << dleg_y[i] << std::endl;
          std::cerr << i << " t_x " << m_force[i].data[3] << " "<< tau_xl[i] << std::endl;
          std::cerr << i << " t_y " << m_force[i].data[4] << " "<< tau_yl[i] << std::endl;
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
      dpz = m_f_z.update((m_force[0].data[2] - m_force[1].data[2]), refdfz);
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
        //manip2[i]->solveLimbIK(vel_p, vel_r, transition_count, 0.001, 0.01, MAX_TRANSITION_COUNT, qrefv, DEBUGP);
        //manip2[i]->solveLimbIK(vel_p, vel_r, transition_count, 0.001, 0.01, MAX_TRANSITION_COUNT, qrefv, false);
        //m_robot->joint(m_robot->link(target_name[i])->jointId)->q = dleg_y[i] + orgjq;
      }
      // m_robot->joint(m_robot->link("L_ANKLE_P")->jointId)->q = transition_smooth_gain * dleg_y[0] + orgjq + m_rpy.data.p;
      // m_robot->joint(m_robot->link("R_ANKLE_P")->jointId)->q = transition_smooth_gain * dleg_y[1] + orgjq + m_rpy.data.p;
      m_robot->joint(m_robot->link("L_ANKLE_P")->jointId)->q = transition_smooth_gain * dleg_y[0] + orgjq;
      m_robot->joint(m_robot->link("R_ANKLE_P")->jointId)->q = transition_smooth_gain * dleg_y[1] + orgjq;
    } else {
      // reinitialize
      for (int i = 0; i < ST_NUM_LEGS; i++) {
        m_tau_x[i].reset();
        m_tau_y[i].reset();
        m_f_z.reset();
      }
    }
  }
}

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
  pangx_ref = pangy_ref = pangx = pangy = 0;
  rdx = rdy = rx = ry = 0;
  d_rpy[0] = d_rpy[1] = 0;
  pdr = hrp::Vector3::Zero();
  zctrl = f_zctrl[0] = f_zctrl[1] = 0.0;
  d_foot_rpy[0] = d_foot_rpy[1] = hrp::Vector3::Zero();
  if (on_ground) {
    transition_count = -MAX_TRANSITION_COUNT;
    control_mode = MODE_ST;
  } else {
    transition_count = 0;
    control_mode = MODE_AIR;
  }
}

void Stabilizer::sync_2_idle ()
{
  transition_count = MAX_TRANSITION_COUNT;
  for (int i = 0; i < m_robot->numJoints(); i++ ) {
    transition_joint_q[i] = m_robot->joint(i)->q;
  }
}

void Stabilizer::startStabilizer(void)
{
  if ( transition_count == 0 && control_mode == MODE_IDLE ) {
    std::cerr << "START ST"  << std::endl;
    sync_2_st();
    waitSTTransition();
    std::cerr << "START ST DONE"  << std::endl;
  }
}

void Stabilizer::stopStabilizer(void)
{
  if ( transition_count == 0 && (control_mode == MODE_ST || control_mode == MODE_AIR) ) {
    std::cerr << "STOP ST" << std::endl;
    control_mode = MODE_SYNC_TO_IDLE;
    waitSTTransition();
    std::cerr << "STOP ST DONE"  << std::endl;
  }
}

void Stabilizer::getParameter(OpenHRP::StabilizerService::stParam& i_stp)
{
  for (size_t i = 0; i < 2; i++) {
    i_stp.k_run_b[i] = k_run_b[i];
    i_stp.d_run_b[i] = d_run_b[i];
    //m_tau_x[i].setup(i_stp.tdfke[0], i_stp.tdftc[0], dt);
    //m_tau_y[i].setup(i_stp.tdfke[0], i_stp.tdftc[0], dt);
    //m_f_z.setup(i_stp.tdfke[1], i_stp.tdftc[1], dt);
    i_stp.k_tpcc_p[i] = k_tpcc_p[i];
    i_stp.k_tpcc_x[i] = k_tpcc_x[i];
    i_stp.k_brot_p[i] = k_brot_p[i];
    i_stp.k_brot_tc[i] = k_brot_tc[i];
  }
  i_stp.k_run_x = m_torque_k[0];
  i_stp.k_run_y = m_torque_k[1];
  i_stp.d_run_x = m_torque_d[0];
  i_stp.d_run_y = m_torque_d[1];
  for (size_t i = 0; i < 2; i++) {
    i_stp.eefm_k1[i] = eefm_k1[i];
    i_stp.eefm_k2[i] = eefm_k2[i];
    i_stp.eefm_k3[i] = eefm_k3[i];
    i_stp.eefm_zmp_delay_time_const[i] = eefm_zmp_delay_time_const[i];
    i_stp.eefm_ref_zmp_aux[i] = ref_zmp_aux(i);
  }
  i_stp.eefm_rot_damping_gain = eefm_rot_damping_gain;
  i_stp.eefm_pos_damping_gain = eefm_pos_damping_gain;
  i_stp.eefm_rot_time_const = eefm_rot_time_const;
  i_stp.eefm_pos_time_const = eefm_pos_time_const;
};

void Stabilizer::setParameter(const OpenHRP::StabilizerService::stParam& i_stp)
{
  for (size_t i = 0; i < 2; i++) {
    k_run_b[i] = i_stp.k_run_b[i];
    d_run_b[i] = i_stp.d_run_b[i];
    m_tau_x[i].setup(i_stp.tdfke[0], i_stp.tdftc[0], dt);
    m_tau_y[i].setup(i_stp.tdfke[0], i_stp.tdftc[0], dt);
    m_f_z.setup(i_stp.tdfke[1], i_stp.tdftc[1], dt);
    k_tpcc_p[i] = i_stp.k_tpcc_p[i];
    k_tpcc_x[i] = i_stp.k_tpcc_x[i];
    k_brot_p[i] = i_stp.k_brot_p[i];
    k_brot_tc[i] = i_stp.k_brot_tc[i];
    std::cerr << i << " k_run_b " << k_run_b[i] << " d_run_b " << d_run_b[i] << std::endl;
    std::cerr << i << " m_tau_xy " << i_stp.tdfke[i] << " " << i_stp.tdftc[i] << std::endl;
    std::cerr << i << " k_tpcc_p " << k_tpcc_p[i] << " k_tpcc_x " << k_tpcc_x[i] << std::endl;
    std::cerr << i << " k_brot_p " << k_brot_p[i] << " k_brot_tc " << k_brot_tc[i] << std::endl;
  }
  m_torque_k[0] = i_stp.k_run_x;
  m_torque_k[1] = i_stp.k_run_y;
  m_torque_d[0] = i_stp.d_run_x;
  m_torque_d[1] = i_stp.d_run_y;
  std::cerr << " m_torque_k " << m_torque_k[0] << " m_torque_k " <<  m_torque_k[1] << std::endl;
  std::cerr << " m_torque_d " << m_torque_d[0] << " m_torque_d " <<  m_torque_d[1] << std::endl;
  for (size_t i = 0; i < 2; i++) {
    eefm_k1[i] = i_stp.eefm_k1[i];
    eefm_k2[i] = i_stp.eefm_k2[i];
    eefm_k3[i] = i_stp.eefm_k3[i];
    eefm_zmp_delay_time_const[i] = i_stp.eefm_zmp_delay_time_const[i];
    ref_zmp_aux(i) = i_stp.eefm_ref_zmp_aux[i];
    std::cerr << i << " eefm_k1 " << eefm_k1[i] << " eefm_k2 " <<  eefm_k2[i] << " eefm_k3 " << eefm_k3[i] << " " << eefm_zmp_delay_time_const[i] << " " << ref_zmp_aux(i) << std::endl;
  }
  eefm_rot_damping_gain = i_stp.eefm_rot_damping_gain;
  eefm_pos_damping_gain = i_stp.eefm_pos_damping_gain;
  eefm_rot_time_const = i_stp.eefm_rot_time_const;
  eefm_pos_time_const = i_stp.eefm_pos_time_const;
  std::cerr << " eefm_rot_damping_gain " << eefm_rot_damping_gain << " eefm_rot_time_const " <<  eefm_rot_time_const << std::endl;
  std::cerr << " eefm_pos_damping_gain " << eefm_pos_damping_gain << " eefm_pos_time_const " <<  eefm_pos_time_const << std::endl;
}

void Stabilizer::waitSTTransition()
{
  while (transition_count != 0) usleep(10);
  usleep(10);
}

static double vlimit(double value, double llimit_value, double ulimit_value)
{
  if (value > ulimit_value) {
    return ulimit_value;
  } else if (value < llimit_value) {
    return llimit_value;
  }
  return value;
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


