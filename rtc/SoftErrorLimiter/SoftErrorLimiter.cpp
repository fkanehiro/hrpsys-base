// -*- C++ -*-
/*!
 * @file  SoftErrorLimiter.cpp
 * @brief soft error limiter
 * $Date$
 *
 * $Id$
 */

#include "SoftErrorLimiter.h"
#include "util/VectorConvert.h"
#include <rtm/CorbaNaming.h>
#include <hrpModel/ModelLoaderUtil.h>
#include "RobotHardwareService.hh"

#include <math.h>
#include <vector>
#define deg2rad(x)((x)*M_PI/180)

#include "beep.h"

// Module specification
// <rtc-template block="module_spec">
static const char* softerrorlimiter_spec[] =
  {
    "implementation_id", "SoftErrorLimiter",
    "type_name",         "SoftErrorLimiter",
    "description",       "soft error limiter",
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

SoftErrorLimiter::SoftErrorLimiter(RTC::Manager* manager)
  : RTC::DataFlowComponentBase(manager),
    // <rtc-template block="initializer">
    m_qRefIn("qRef", m_qRef),
    m_qCurrentIn("qCurrent", m_qCurrent),
    m_servoStateIn("servoStateIn", m_servoState),
    m_qOut("q", m_qRef),
    m_servoStateOut("servoStateOut", m_servoState),
    m_SoftErrorLimiterServicePort("SoftErrorLimiterService"),
    // </rtc-template>
    m_debugLevel(0),
	dummy(0)
{
  init_beep();
  start_beep(3136);
}

SoftErrorLimiter::~SoftErrorLimiter()
{
  quit_beep();
}



RTC::ReturnCode_t SoftErrorLimiter::onInitialize()
{
  std::cout << m_profile.instance_name << ": onInitialize()" << std::endl;
  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  bindParameter("debugLevel", m_debugLevel, "0");
  
  // </rtc-template>

  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("qRef", m_qRefIn);
  addInPort("qCurrent", m_qCurrentIn);
  addInPort("servoState", m_servoStateIn);
  
  // Set OutPort buffer
  addOutPort("q", m_qOut);
  addOutPort("servoState", m_servoStateOut);
  
  // Set service provider to Ports
  m_SoftErrorLimiterServicePort.registerProvider("service0", "SoftErrorLimiterService", m_service0);
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  addPort(m_SoftErrorLimiterServicePort);
  
  // </rtc-template>

  m_robot = boost::shared_ptr<robot>(new robot());
  RTC::Properties& prop = getProperties();

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

  std::cout << "dof = " << m_robot->numJoints() << std::endl;
  if (!m_robot->init()) return RTC::RTC_ERROR;
  m_service0.setRobot(m_robot);
  m_servoState.data.length(m_robot->numJoints());
  for(int i = 0; i < m_robot->numJoints(); i++) {
    m_servoState.data[i].length(1);
    int status = 0;
    status |= 1<< OpenHRP::RobotHardwareService::CALIB_STATE_SHIFT;
    status |= 1<< OpenHRP::RobotHardwareService::POWER_STATE_SHIFT;
    status |= 1<< OpenHRP::RobotHardwareService::SERVO_STATE_SHIFT;
    status |= 0<< OpenHRP::RobotHardwareService::SERVO_ALARM_SHIFT;
    status |= 0<< OpenHRP::RobotHardwareService::DRIVER_TEMP_SHIFT;
    m_servoState.data[i][0] = status;
  }

  /* Calculate count for beep sound frequency */
  coil::stringTo(dt, prop["dt"].c_str());
  soft_limit_error_beep_freq = static_cast<int>(1.0/(4.0*dt)); // soft limit error => 4 times / 1[s]
  position_limit_error_beep_freq = static_cast<int>(1.0/(2.0*dt)); // position limit error => 2 times / 1[s]
  debug_print_freq = static_cast<int>(0.02/dt); // once per 0.02 [s]
  /* If you print debug message for all controller loop, please comment in here */
  // debug_print_freq = 1;

  // load joint limit table
  hrp::readJointLimitTableFromProperties (joint_limit_tables, m_robot, prop["joint_limit_table"], std::string(m_profile.instance_name));

  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t SoftErrorLimiter::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t SoftErrorLimiter::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t SoftErrorLimiter::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

RTC::ReturnCode_t SoftErrorLimiter::onActivated(RTC::UniqueId ec_id)
{
  std::cout << m_profile.instance_name<< ": onActivated(" << ec_id << ")" << std::endl;
  return RTC::RTC_OK;
}

RTC::ReturnCode_t SoftErrorLimiter::onDeactivated(RTC::UniqueId ec_id)
{
  std::cout << m_profile.instance_name<< ": onDeactivated(" << ec_id << ")" << std::endl;
  return RTC::RTC_OK;
}

RTC::ReturnCode_t SoftErrorLimiter::onExecute(RTC::UniqueId ec_id)
{
  //std::cout << m_profile.instance_name<< ": onExecute(" << ec_id << ")" << std::endl;
  static int loop = 0;
  loop ++;

  if (m_qRefIn.isNew()) {
    m_qRefIn.read();
  }
  if (m_qCurrentIn.isNew()) {
    m_qCurrentIn.read();
  }
  if (m_servoStateIn.isNew()) {
    m_servoStateIn.read();
  }

  /*
    0x001 : 'SS_OVER_VOLTAGE',
    0x002 : 'SS_OVER_LOAD',
    0x004 : 'SS_OVER_VELOCITY',
    0x008 : 'SS_OVER_CURRENT',
    0x010 : 'SS_OVER_HEAT',
    0x020 : 'SS_TORQUE_LIMIT',
    0x040 : 'SS_VELOCITY_LIMIT',
    0x080 : 'SS_FORWARD_LIMIT',
    0x100 : 'SS_REVERSE_LIMIT',
    0x200 : 'SS_POSITION_ERROR',
    0x300 : 'SS_ENCODER_ERROR',
    0x800 : 'SS_OTHER'
  */
  bool soft_limit_error = false;
  bool velocity_limit_error = false;
  bool position_limit_error = false;
  if ( m_qRef.data.length() == m_qCurrent.data.length() &&
       m_qRef.data.length() == m_servoState.data.length() ) {
    static std::vector<double> prev_angle;
    if ( prev_angle.size() != m_qRef.data.length() ) { // initialize prev_angle
      prev_angle.resize(m_qRef.data.length(), 0);
      for ( int i = 0; i < m_qRef.data.length(); i++ ){
        prev_angle[i] = m_qCurrent.data[i];
      }
    }
    std::vector<int> servo_state;
    servo_state.resize(m_qRef.data.length(), 0);
    for ( int i = 0; i < m_qRef.data.length(); i++ ){
        servo_state[i] = (m_servoState.data[i][0] & OpenHRP::RobotHardwareService::SERVO_STATE_MASK) >> OpenHRP::RobotHardwareService::SERVO_STATE_SHIFT; // enum SwitchStatus {SWITCH_ON, SWITCH_OFF};
    }

    // Velocity limitation for reference joint angles
    for ( int i = 0; i < m_qRef.data.length(); i++ ){
      double qvel = (m_qRef.data[i] - prev_angle[i]) / dt;
      double lvlimit = m_robot->joint(i)->lvlimit;
      double uvlimit = m_robot->joint(i)->uvlimit;
      if ( servo_state[i] == 1 && ((lvlimit > qvel) || (uvlimit < qvel)) ) {
        if (loop % debug_print_freq == 0) {
          std::cerr << "velocity limit over " << m_robot->joint(i)->name << "(" << i << "), qvel=" << qvel
                    << ", lvlimit =" << lvlimit
                    << ", uvlimit =" << uvlimit
                    << ", servo_state = " <<  ( servo_state[i] ? "ON" : "OFF") << std::endl;
        }
        // fix joint angle
        if ( lvlimit > qvel ) m_qRef.data[i] = prev_angle[i] + lvlimit * dt;
        if ( uvlimit < qvel ) m_qRef.data[i] = prev_angle[i] + uvlimit * dt;
        velocity_limit_error = true;
      }
    }

    // Position limitation for reference joint angles
    for ( int i = 0; i < m_qRef.data.length(); i++ ){
      double error = m_qRef.data[i] - m_qCurrent.data[i];
      /*
        From hrpModel/Body.h
        inline Link* joint(int id) const
           This function returns a link that has a given joint ID.
           If there is no link that has a given joint ID,
           the function returns a dummy link object whose ID is minus one.
           The maximum id can be obtained by numJoints().

         inline Link* link(int index) const
           This function returns the link of a given index in the whole link sequence.
           The order of the sequence corresponds to a link-tree traverse from the root link.
           The size of the sequence can be obtained by numLinks().

         So use m_robot->joint(i) for llimit/ulimit
       */
      double llimit = m_robot->joint(i)->llimit;
      double ulimit = m_robot->joint(i)->ulimit;
      if (joint_limit_tables.find(m_robot->joint(i)->name) != joint_limit_tables.end()) {
          std::map<std::string, hrp::JointLimitTable>::iterator it = joint_limit_tables.find(m_robot->joint(i)->name);
          llimit = it->second.getLlimit(m_qRef.data[it->second.getTargetJointId()]);
          ulimit = it->second.getUlimit(m_qRef.data[it->second.getTargetJointId()]);
      }
      bool servo_limit_state = ((llimit > m_qRef.data[i]) || (ulimit < m_qRef.data[i]));
      if ( servo_state[i] == 1 && servo_limit_state ) {
        if (loop % debug_print_freq == 0) {
          std::cerr << "position limit over " << m_robot->joint(i)->name << "(" << i << "), qRef=" << m_qRef.data[i]
                    << ", llimit =" << llimit
                    << ", ulimit =" << ulimit
                    << ", servo_state = " <<  ( servo_state[i] ? "ON" : "OFF")
                    << ", prev_angle = " << prev_angle[i] << std::endl;
        }
        // fix joint angle
        if ( llimit > m_qRef.data[i] && prev_angle[i] > m_qRef.data[i] ) // ref < llimit and prev < ref -> OK
          m_qRef.data[i] = prev_angle[i];
        if ( ulimit < m_qRef.data[i] && prev_angle[i] < m_qRef.data[i] ) // ulimit < ref and ref < prev -> OK
          m_qRef.data[i] = prev_angle[i];
        m_servoState.data[i][0] |= (0x200 << OpenHRP::RobotHardwareService::SERVO_ALARM_SHIFT);
        position_limit_error = true;
      }
      prev_angle[i] = m_qRef.data[i];
    }

    // Servo error limitation between reference joint angles and actual joint angles
    for ( int i = 0; i < m_qRef.data.length(); i++ ){
      double limit = m_robot->m_servoErrorLimit[i];
      double error = m_qRef.data[i] - m_qCurrent.data[i];
      if ( servo_state[i] == 1 && fabs(error) > limit ) {
        if (loop % debug_print_freq == 0) {
          std::cerr << "error limit over " << m_robot->joint(i)->name << "(" << i << "), qRef=" << m_qRef.data[i]
                    << ", qCurrent=" << m_qCurrent.data[i] << " "
                    << ", Error=" << error << " > " << limit << " (limit)"
                    << ", servo_state = " <<  ( 1 ? "ON" : "OFF");
        }
        m_qRef.data[i] = m_qCurrent.data[i] + ( error > 0 ? limit : -limit );
        if (loop % debug_print_freq == 0) {
          std::cerr << ", q=" << m_qRef.data[i] << std::endl;
        }
        m_servoState.data[i][0] |= (0x040 << OpenHRP::RobotHardwareService::SERVO_ALARM_SHIFT);
        soft_limit_error = true;
      }
    }

    // Beep sound
    if ( soft_limit_error ) { // play beep
      if ( loop % soft_limit_error_beep_freq == 0 ) start_beep(3136, soft_limit_error_beep_freq*0.8);
    }else if ( position_limit_error || velocity_limit_error ) { // play beep
      if ( loop % position_limit_error_beep_freq == 0 ) start_beep(3520, position_limit_error_beep_freq*0.8);
    } else {
      stop_beep();
    }
    m_qOut.write();
    m_servoStateOut.write();
  } else {
    start_beep(3136);
    if ( loop % 100 == 1 ) {
        std::cerr << "SoftErrorLimiter is not working..." << std::endl;
        std::cerr << "         m_qRef " << m_qRef.data.length() << std::endl;
        std::cerr << "     m_qCurrent " << m_qCurrent.data.length() << std::endl;
        std::cerr << "   m_servoState " << m_servoState.data.length() << std::endl;
    }
  }

  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t SoftErrorLimiter::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t SoftErrorLimiter::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t SoftErrorLimiter::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t SoftErrorLimiter::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t SoftErrorLimiter::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/



extern "C"
{

  void SoftErrorLimiterInit(RTC::Manager* manager)
  {
    RTC::Properties profile(softerrorlimiter_spec);
    manager->registerFactory(profile,
                             RTC::Create<SoftErrorLimiter>,
                             RTC::Delete<SoftErrorLimiter>);
  }

};


