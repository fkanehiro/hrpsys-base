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
    "version",           "1.0",
    "vendor",            "AIST",
    "category",          "example",
    "activity_type",     "DataFlowComponent",
    "max_instance",      "10",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    "conf.default.debugLevel", "1",
    ""
  };
// </rtc-template>

static double limit(double value, double limit_value);
static double switching_inpact_absorber(double force, double lower_th, double upper_th);

Stabilizer::Stabilizer(RTC::Manager* manager)
  : RTC::DataFlowComponentBase(manager),
    // <rtc-template block="initializer">
    m_qIn("qCurrent", m_q),
    m_qRefIn("qRef", m_qRef),
    m_rpyIn("rpy", m_rpy),
    m_forceLIn("forceL", m_force[ST_LEFT]),
    m_forceRIn("forceR", m_force[ST_RIGHT]),
    m_zmpRefIn("zmpRef", m_zmpRef),
    m_accRefIn("accRef", m_accRef),
    m_rpyRefIn("rpyRef", m_rpyRef),
    m_qRefOut("q", m_qRef),
    m_StabilizerServicePort("StabilizerService"),
    m_isExecute(false),
    // </rtc-template>
    m_debugLevel(1)
{
}

Stabilizer::~Stabilizer()
{
}

RTC::ReturnCode_t Stabilizer::onInitialize()
{
  std::cerr << m_profile.instance_name << ": onInitialize()" << std::endl;
  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  bindParameter("debugLevel", m_debugLevel, "1");
  
  // </rtc-template>

  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("q", m_qIn);
  addInPort("qRef", m_qRefIn);
  addInPort("forceR", m_forceRIn);
  addInPort("forceL", m_forceLIn);
  addInPort("rpy", m_rpyIn);
  addInPort("zmpRef", m_zmpRefIn);
  addInPort("rpyRef", m_rpyRefIn);

  // Set OutPort buffer
  addOutPort("qRef", m_qRefOut);
  
  // Set service provider to Ports
  m_StabilizerServicePort.registerProvider("service0", "StabilizerService", m_StabilizerService);
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  addPort(m_StabilizerServicePort);
  
  // </rtc-template>
  RTC::Properties& prop = getProperties();
  double ke = 0, tc = 0, dt = 0;

  // parameters for two dof torque controller
  coil::stringTo(ke, prop["torque_controller_ke"].c_str());
  coil::stringTo(tc,  prop["torque_controller_tc"].c_str());
  coil::stringTo(dt, prop["torque_controller_dt"].c_str());
  if ( ke == 0 || tc == 0 || dt == 0 ) {
    std::cerr << "Stabilizer: Torque controll parameter(ke,tc,dt) is not set" << std::endl;
    //return RTC::RTC_ERROR;
  }else{
    for (int i = 0; i < ST_NUM_LEGS; i++) {
      m_tau_pitch[i].setup(ke, tc, dt);
      m_tau_roll[i].setup(ke, tc, dt);
    }
  }
  if (m_debugLevel > 0) {
    std::cout << "ke: " << ke << std::endl;
    std::cout << "tc: " << tc << std::endl;
    std::cout << "dt: " << dt << std::endl; 
  }

  // parameters for 3D-LIP feedback controller 
  coil::vstring lip_k = coil::split(prop["lip_k"], ",");
  coil::vstring lip_d = coil::split(prop["lip_d"], ",");
  if (lip_k.size() == 2 && lip_d.size() == 2) { // x, y
    for (int i = 0; i < 2; i++) { // x, y
      coil::stringTo(m_torque_k[i], lip_k[i].c_str());
      coil::stringTo(m_torque_d[i], lip_d[i].c_str());
    }
  }else{
    std::cerr << "Stabilizer: 3D-LIP feedback parameters are not set" << std::endl;
    //return RTC::RTC_ERROR;
  }
  if (m_debugLevel > 0) {
    for (int i = 0; i < 2; i++) { // x, y
      std::cout << "3d-lip[" << i << "]: " << m_torque_k[i] << " " << m_torque_d[i] << std::endl;
    }
  }
  
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
    std::cerr << "failed to load model[" << prop["model"] << "]" 
              << std::endl;
    return RTC::RTC_ERROR;
  }
  m_robot->totalMass();

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

RTC::ReturnCode_t Stabilizer::onExecute(RTC::UniqueId ec_id)
{
  // std::cout << m_profile.instance_name<< ": onExecute(" << ec_id << ")" << std::endl;

  if (m_qRefIn.isNew()) {
    m_qRefIn.read();
  }
  if (m_qIn.isNew()) {
    m_qIn.read();
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

  if ( m_robot->numJoints() == m_qRef.data.length() ) {

    // update internal robot model
    for ( int i = 0; i < m_robot->numJoints(); i++ ){
      m_robot->joint(i)->q = m_qRef.data[i];
    }
    m_robot->calcForwardKinematics();

    if (m_debugLevel > 0) {
      std::cout << "st-sensor-state" << std::endl;
      for (int i = 0; i < ST_NUM_LEGS; i++) {
        std::cout << m_force[i].data.length() << std::endl;
        if (m_force[i].data.length() > 0) {
          std::cout << m_force[i].data[2] << ">" << m_robot->totalMass()/4 << std::endl;
          std::cout << m_force[i].data[3] << " " << m_force[i].data[4] << std::endl;
        }
      }
      std::cout << std::endl;
    }

    // stabilizer loop
    if ( ( m_force[ST_LEFT].data.length() > 0 && m_force[ST_RIGHT].data.length() > 0 ) &&
         ( m_force[ST_LEFT].data[2] > m_robot->totalMass()/4 || m_force[ST_RIGHT].data[2] > m_robot->totalMass()/4 ) ) {

      // midcoords (L_FOOT, R_FOOT)
      hrp::Vector3 p = (m_robot->link("L_FOOT")->p + m_robot->link("R_FOOT")->p)/2;
      hrp::Vector3 omega = (hrp::omegaFromRot(m_robot->link("L_FOOT")->R) + hrp::omegaFromRot(m_robot->link("R_FOOT")->R))/2;
      hrp::Matrix33 R = hrp::rodrigues(omega.normalized(), omega.norm());
      //std::cerr << "p = " << p << ", omega = " << omega << ", R = " << R << " cm = " << m_robot->calcCM() << std::endl;
      // ivnerse-transfrom-vector(CM)
      hrp::Vector3 cm = R.transpose() * ( m_robot->calcCM() - p); // vector from centor of foot to center of mass 
      if ( m_debugLevel > 0 ) {
        std::cerr << "cm = (" << cm[0] << " " << cm[1] << " " << cm[2] << ")" << std::endl;
      }

      // 3D-LIP model contorller
      static double old_error[2];
      double diff_error[2];
      double tau_pitch, tau_roll;
      for (int i = 0; i < 2; i++) { // x, y
        diff_error[i] = cm[i] - old_error[i];
        old_error[i] = cm[i];
      }
      // dist: dx = dy = 0
      tau_pitch = - m_torque_k[ST_X] * cm[ST_X] - m_torque_d[ST_X] * diff_error[ST_X]; // pitch(tau_y)
      tau_roll  =   m_torque_k[ST_Y] * cm[ST_Y] + m_torque_d[ST_Y] * diff_error[ST_Y]; // roll(tau_x)

      // torque distribution
      // double dist_force_z = m_robot->totalMass() * 9.8 /2;
      // double total_force_z = m_robot->totalMass() * 9.8;
      // double diff_force_z = 0;
      // double feet_interval = (m_robot->link("L_FOOT")->p + m_robot->link("R_FOOT")->p).norm();
      // double reference_force_z, ratio;
      // double dist_tau_pitch[ST_NUM_LEGS], dist_tau_roll[ST_NUM_LEGS];
      // reference_force_z = limit(2 * tau_roll / feet_interval + diff_force_z, total_force_z);
      // ratio = (total_force_z - reference_force_z) / (2 * total_force_z);
      // dist_tau_pitch[ST_LEFT] = (1 - ratio) * tau_pitch;
      // dist_tau_roll[ST_LEFT] = (1 - ratio) * (tau_roll - feet_interval * (reference_force_z - diff_force_z) / 2);
      // dist_tau_pitch[ST_RIGHT] = ratio * tau_pitch;
      // dist_tau_roll[ST_RIGHT] = ratio * (tau_roll - feet_interval * (reference_force_z - diff_force_z) / 2);
      // if ( m_debugLevel > 0 ) {
      //   std::cerr << "ratio: " << ratio << std::endl;
      //   std::cerr << "reference_force: " << reference_force_z << std::endl;
      // }
      double dist_tau_pitch[ST_NUM_LEGS], dist_tau_roll[ST_NUM_LEGS];
      dist_tau_pitch[ST_LEFT] = tau_pitch / 2;
      dist_tau_roll[ST_LEFT] = tau_roll / 2;
      dist_tau_pitch[ST_RIGHT] = tau_pitch / 2;
      dist_tau_roll[ST_RIGHT] = tau_roll / 2;
        
      // torque controll
      double dq_pitch[ST_NUM_LEGS], dq_roll[ST_NUM_LEGS];
      for (int i = 0; i < ST_NUM_LEGS; i++) {
        dq_pitch[i] = m_tau_pitch[i].update(m_force[i].data[4], dist_tau_pitch[i]);
        dq_roll[i] = m_tau_roll[i].update(m_force[i].data[3], dist_tau_roll[i]);
      }
      m_qRef.data[m_robot->link("L_ANKLE_P")->jointId] += limit(dq_pitch[ST_LEFT], ST_MAX_DQ);
      m_qRef.data[m_robot->link("R_ANKLE_P")->jointId] += limit(dq_pitch[ST_RIGHT], ST_MAX_DQ);
      m_qRef.data[m_robot->link("L_ANKLE_R")->jointId] += limit(dq_roll[ST_LEFT], ST_MAX_DQ);
      m_qRef.data[m_robot->link("R_ANKLE_R")->jointId] += limit(dq_roll[ST_RIGHT], ST_MAX_DQ);
      if ( m_debugLevel > 0 ) {
        std::cerr << "tau: p = " << tau_pitch << ", r = " << tau_roll << std::endl;
        std::cerr << "force: p = " << m_force[ST_LEFT].data[4] << "  " << m_force[ST_RIGHT].data[4] 
                  << ", r = " << m_force[ST_LEFT].data[3] << "  " << m_force[ST_RIGHT].data[3] << std::endl;
        std::cerr << "left_ankle_q: dp = " << dq_pitch[ST_LEFT] << ", dr = " << dq_roll[ST_LEFT]
                  << ", p = " << m_qRef.data[m_robot->link("L_ANKLE_P")->jointId] << ", r = " << m_qRef.data[m_robot->link("L_ANKLE_R")->jointId] << std::endl;
        std::cerr << "right_ankle_q: dp = " << dq_pitch[ST_RIGHT] << ", dr = " << dq_roll[ST_RIGHT]
                  << ", p = " << m_qRef.data[m_robot->link("R_ANKLE_P")->jointId] << ", r = " << m_qRef.data[m_robot->link("R_ANKLE_R")->jointId] << std::endl;
      }
    } else {
      // reinitialize
      for (int i = 0; i < ST_NUM_LEGS; i++) {
        m_tau_pitch[i].reset();
        m_tau_roll[i].reset();
      }
    }

    m_qRefOut.write();
  }
  return RTC::RTC_OK;
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

void Stabilizer::startStabilizer(void)
{
  m_isExecute = true;
}

void Stabilizer::stopStabilizer(void)
{
  m_isExecute = false;
}

static double limit(double value, double limit_value)
{
  if (value > limit_value) {
    return limit_value;
  } else if (value < -limit_value) {
    return -limit_value;
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


