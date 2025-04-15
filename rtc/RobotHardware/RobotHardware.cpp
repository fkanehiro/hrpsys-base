// -*- C++ -*-
/*!
 * @file  RobotHardware.cpp
 * @brief null component
 * $Date$
 *
 * $Id$
 */

#include <rtm/CorbaNaming.h>
#include "hrpsys/util/VectorConvert.h"
#include "RobotHardware.h"
#include "robot.h"

#include <hrpModel/Sensor.h>
#include <hrpModel/ModelLoaderUtil.h>

using namespace OpenHRP;
using namespace hrp;

// Module specification
// <rtc-template block="module_spec">
static const char* robothardware_spec[] =
  {
    "implementation_id", "RobotHardware",
    "type_name",         "RobotHardware",
    "description",       "RobotHardware",
    "version",           HRPSYS_PACKAGE_VERSION,
    "vendor",            "AIST",
    "category",          "example",
    "activity_type",     "DataFlowComponent",
    "max_instance",      "1",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    "conf.default.isDemoMode", "0",
    "conf.default.fzLimitRatio", "2.0",
    "conf.default.servoErrorLimit", ",",
    "conf.default.jointAccelerationLimit", "0",
    "conf.default.servoOnDelay", "0",

    ""
  };
// </rtc-template>

RobotHardware::RobotHardware(RTC::Manager* manager)
  : RTC::DataFlowComponentBase(manager),
    // <rtc-template block="initializer">
    m_isDemoMode(0),
    m_qRefIn("qRef", m_qRef),
    m_dqRefIn("dqRef", m_dqRef),
    m_ddqRefIn("ddqRef", m_ddqRef),
    m_tauRefIn("tauRef", m_tauRef),
    m_qOut("q", m_q),
    m_dqOut("dq", m_dq),
    m_tauOut("tau", m_tau),
    m_ctauOut("ctau", m_ctau),
    m_pdtauOut("pdtau", m_pdtau),
    m_servoStateOut("servoState", m_servoState),
    m_pgainOut("pgain", m_pgain),
    m_dgainOut("dgain", m_dgain),
    m_torquePgainOut("torquePgain", m_torquePgain),
    m_torqueDgainOut("torqueDgain", m_torqueDgain),
    m_emergencySignalOut("emergencySignal", m_emergencySignal),
    m_rstate2Out("rstate2", m_rstate2),
    m_RobotHardwareServicePort("RobotHardwareService"),
    // </rtc-template>
	dummy(0)
{
}

RobotHardware::~RobotHardware()
{
}


RTC::ReturnCode_t RobotHardware::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  
  addInPort("qRef", m_qRefIn);
  addInPort("dqRef", m_dqRefIn);
  addInPort("ddqRef", m_ddqRefIn);
  addInPort("tauRef", m_tauRefIn);

  addOutPort("q", m_qOut);
  addOutPort("dq", m_dqOut);
  addOutPort("tau", m_tauOut);
  addOutPort("ctau", m_ctauOut);
  addOutPort("pdtau", m_pdtauOut);
  addOutPort("servoState", m_servoStateOut);
  addOutPort("pgain", m_pgainOut);
  addOutPort("dgain", m_dgainOut);
  addOutPort("torquePgain", m_torquePgainOut);
  addOutPort("torqueDgain", m_torqueDgainOut);
  addOutPort("emergencySignal", m_emergencySignalOut);
  addOutPort("rstate2", m_rstate2Out);

  // Set service provider to Ports
    m_RobotHardwareServicePort.registerProvider("service0", "RobotHardwareService", m_service0);
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  addPort(m_RobotHardwareServicePort);
  
  // </rtc-template>

  RTC::Properties& prop = getProperties();
  double dt = 0.0;
  coil::stringTo(dt, prop["dt"].c_str());
  if (!dt) {
      std::cerr << m_profile.instance_name << ": joint command velocity check is disabled" << std::endl;
  }
  m_robot = boost::shared_ptr<robot>(new robot(dt));


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
      if (prop["model"] == ""){
          std::cerr << "[" << m_profile.instance_name << "] path to the model file is not defined. Please check the configuration file" << std::endl;
      }else{
          std::cerr << "[" << m_profile.instance_name << "] failed to load model[" << prop["model"] << "]" << std::endl;
      }
  }

  std::vector<std::string> keys = prop.propertyNames();
  for (unsigned int i=0; i<keys.size(); i++){
      m_robot->setProperty(keys[i].c_str(), prop[keys[i]].c_str());
  } 
  std::cout << "dof = " << m_robot->numJoints() << std::endl;
  if (!m_robot->init()) return RTC::RTC_ERROR;

  m_service0.setRobot(m_robot);

  m_q.data.length(m_robot->numJoints());
  m_dq.data.length(m_robot->numJoints());
  m_tau.data.length(m_robot->numJoints());
  m_ctau.data.length(m_robot->numJoints());
  m_pdtau.data.length(m_robot->numJoints());
  m_servoState.data.length(m_robot->numJoints());
  m_pgain.data.length(m_robot->numJoints());
  m_dgain.data.length(m_robot->numJoints());
  m_torquePgain.data.length(m_robot->numJoints());
  m_torqueDgain.data.length(m_robot->numJoints());
  m_qRef.data.length(m_robot->numJoints());
  m_dqRef.data.length(m_robot->numJoints());
  m_ddqRef.data.length(m_robot->numJoints());
  m_tauRef.data.length(m_robot->numJoints());

  int ngyro = m_robot->numSensors(Sensor::RATE_GYRO);
  std::cout << "the number of gyros = " << ngyro << std::endl;
  m_rate.resize(ngyro);
  m_rateOut.resize(ngyro);
  for (unsigned int i=0; i<m_rate.size(); i++){
      Sensor *s = m_robot->sensor(Sensor::RATE_GYRO, i);
      std::cout << s->name << std::endl;
      m_rateOut[i] = new OutPort<TimedAngularVelocity3D>(s->name.c_str(), m_rate[i]);
      registerOutPort(s->name.c_str(), *m_rateOut[i]);
  }

  int nacc = m_robot->numSensors(Sensor::ACCELERATION);
  std::cout << "the number of accelerometers = " << nacc << std::endl;
  m_acc.resize(nacc);
  m_accOut.resize(nacc);
  for (unsigned int i=0; i<m_acc.size(); i++){
      Sensor *s = m_robot->sensor(Sensor::ACCELERATION, i);
      std::cout << s->name << std::endl;
      m_accOut[i] = new OutPort<TimedAcceleration3D>(s->name.c_str(), m_acc[i]);
      registerOutPort(s->name.c_str(), *m_accOut[i]);
  }

  int nforce = m_robot->numSensors(Sensor::FORCE);
  std::cout << "the number of force sensors = " << nforce << std::endl;
  m_force.resize(nforce);
  m_forceOut.resize(nforce);
  for (unsigned int i=0; i<m_force.size(); i++){
      Sensor *s = m_robot->sensor(Sensor::FORCE, i);
      std::cout << s->name << std::endl;
      m_forceOut[i] = new OutPort<TimedDoubleSeq>(s->name.c_str(), m_force[i]);
      m_force[i].data.length(6);
      registerOutPort(s->name.c_str(), *m_forceOut[i]);
  }

  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  bindParameter("isDemoMode", m_isDemoMode, "0");  
  bindParameter("servoErrorLimit", m_robot->m_servoErrorLimit, ",");
  bindParameter("fzLimitRatio", m_robot->m_fzLimitRatio, "2");
  bindParameter("jointAccelerationLimit", m_robot->m_accLimit, "0");
  bindParameter("servoOnDelay", m_robot->m_servoOnDelay, "0");

  // </rtc-template>

  return RTC::RTC_OK;
}


/*
RTC::ReturnCode_t RobotHardware::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t RobotHardware::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t RobotHardware::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t RobotHardware::onActivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t RobotHardware::onDeactivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

RTC::ReturnCode_t RobotHardware::onExecute(RTC::UniqueId ec_id)
{
    //std::cout << "RobotHardware:onExecute(" << ec_id << ")" << std::endl;
  Time tm;
  this->getTimeNow(tm);

  if (!m_isDemoMode){
      robot::emg_reason reason;
      int id;
      if (m_robot->checkEmergency(reason, id)){
          if (reason == robot::EMG_SERVO_ERROR || reason == robot::EMG_POWER_OFF){
              m_robot->servo("all", false);
              m_emergencySignal.data = reason;
              m_emergencySignalOut.write();
          } else if (reason == robot::EMG_SERVO_ALARM) {
              m_emergencySignal.data = reason;
              m_emergencySignalOut.write();
          }
      }
  }    

  if (m_qRefIn.isNew()){
      m_qRefIn.read();
      //std::cout << "RobotHardware: qRef[21] = " << m_qRef.data[21] << std::endl;
      if (!m_isDemoMode 
          && m_robot->checkJointCommands(m_qRef.data.get_buffer())){
          m_robot->servo("all", false);
          m_emergencySignal.data = robot::EMG_SERVO_ERROR;
          m_emergencySignalOut.write();
      }else{
          // output to iob
          m_robot->writeJointCommands(m_qRef.data.get_buffer());
      }
  }
  if (m_dqRefIn.isNew()){
      m_dqRefIn.read();
      //std::cout << "RobotHardware: dqRef[21] = " << m_dqRef.data[21] << std::endl;
      // output to iob
      m_robot->writeVelocityCommands(m_dqRef.data.get_buffer());
  }
  if (m_ddqRefIn.isNew()){
      m_ddqRefIn.read();
      //std::cout << "RobotHardware: dqRef[21] = " << m_dqRef.data[21] << std::endl;
      // output to iob
      m_robot->writeAccelerationCommands(m_ddqRef.data.get_buffer());
  }
  if (m_tauRefIn.isNew()){
      m_tauRefIn.read();
      //std::cout << "RobotHardware: tauRef[21] = " << m_tauRef.data[21] << std::endl;
      // output to iob
      m_robot->writeTorqueCommands(m_tauRef.data.get_buffer());
  }

  // read from iob
  m_robot->readJointAngles(m_q.data.get_buffer());  
  m_q.tm = tm;
  m_robot->readJointVelocities(m_dq.data.get_buffer());  
  m_dq.tm = tm;
  m_robot->readJointTorques(m_tau.data.get_buffer());
  m_tau.tm = tm;
  m_robot->readJointCommandTorques(m_ctau.data.get_buffer());
  m_ctau.tm = tm;
  m_robot->readPDControllerTorques(m_pdtau.data.get_buffer());
  m_pdtau.tm = tm;
  for (unsigned int i=0; i<m_rate.size(); i++){
      double rate[3];
      m_robot->readGyroSensor(i, rate);
      m_rate[i].data.avx = rate[0];
      m_rate[i].data.avy = rate[1];
      m_rate[i].data.avz = rate[2];
      m_rate[i].tm = tm;
  }

  for (unsigned int i=0; i<m_acc.size(); i++){
      double acc[3];
      m_robot->readAccelerometer(i, acc);
      m_acc[i].data.ax = acc[0];
      m_acc[i].data.ay = acc[1];
      m_acc[i].data.az = acc[2];
      m_acc[i].tm = tm;
  }

  for (unsigned int i=0; i<m_force.size(); i++){
      m_robot->readForceSensor(i, m_force[i].data.get_buffer());
      m_force[i].tm = tm;
  }
  
  for (unsigned int i=0; i<m_servoState.data.length(); i++){
      size_t len = m_robot->lengthOfExtraServoState(i)+1;
      m_servoState.data[i].length(len);
      int status = 0, v;
      v = m_robot->readCalibState(i);
      status |= v<< OpenHRP::RobotHardwareService::CALIB_STATE_SHIFT;
      v = m_robot->readPowerState(i);
      status |= v<< OpenHRP::RobotHardwareService::POWER_STATE_SHIFT;
      v = m_robot->readServoState(i);
      status |= v<< OpenHRP::RobotHardwareService::SERVO_STATE_SHIFT;
      v = m_robot->readServoAlarm(i);
      status |= v<< OpenHRP::RobotHardwareService::SERVO_ALARM_SHIFT;
      v = m_robot->readDriverTemperature(i);
      status |= v<< OpenHRP::RobotHardwareService::DRIVER_TEMP_SHIFT;
      m_servoState.data[i][0] = status;
      m_robot->readExtraServoState(i, (int *)(m_servoState.data[i].get_buffer()+1));
  }
  m_servoState.tm = tm;

  for (unsigned int i=0; i<m_pgain.data.length(); i++) m_robot->readPgain(i, m_pgain.data[i]);
  m_pgain.tm = tm;
  for (unsigned int i=0; i<m_dgain.data.length(); i++) m_robot->readDgain(i, m_dgain.data[i]);
  m_dgain.tm = tm;
  for (unsigned int i=0; i<m_torquePgain.data.length(); i++) m_robot->readTorquePgain(i, m_torquePgain.data[i]);
  m_torquePgain.tm = tm;
  for (unsigned int i=0; i<m_torqueDgain.data.length(); i++) m_robot->readTorqueDgain(i, m_torqueDgain.data[i]);
  m_torqueDgain.tm = tm;

  getStatus2(m_rstate2.data);
  m_rstate2.tm = tm;
  
  m_robot->oneStep();

  m_qOut.write();
  m_dqOut.write();
  m_tauOut.write();
  m_ctauOut.write();
  m_pdtauOut.write();
  m_servoStateOut.write();
  m_pgainOut.write();
  m_dgainOut.write();
  m_torquePgainOut.write();
  m_torqueDgainOut.write();
  for (unsigned int i=0; i<m_rateOut.size(); i++){
      m_rateOut[i]->write();
  }
  for (unsigned int i=0; i<m_accOut.size(); i++){
      m_accOut[i]->write();
  }
  for (unsigned int i=0; i<m_forceOut.size(); i++){
      m_forceOut[i]->write();
  }
  m_rstate2Out.write();

  return RTC::RTC_OK;
}

template<class T>
void getStatus(boost::shared_ptr<robot> robot, T& rstate)
{
  rstate.angle.length(robot->numJoints());
  robot->readJointAngles(rstate.angle.get_buffer());

  rstate.command.length(robot->numJoints());
  robot->readJointCommands(rstate.command.get_buffer());

  rstate.torque.length(robot->numJoints());
  if (!robot->readJointTorques(rstate.torque.get_buffer())){
    for (unsigned int i=0; i<rstate.torque.length(); i++){
      rstate.torque[i] = 0.0;
    }
  }

  rstate.servoState.length(robot->numJoints());
  int v, status;
  for(unsigned int i=0; i < rstate.servoState.length(); ++i){
    size_t len = robot->lengthOfExtraServoState(i)+1;
    rstate.servoState[i].length(len);
    status = 0;
    v = robot->readCalibState(i);
    status |= v<< OpenHRP::RobotHardwareService::CALIB_STATE_SHIFT;
    v = robot->readPowerState(i);
    status |= v<< OpenHRP::RobotHardwareService::POWER_STATE_SHIFT;
    v = robot->readServoState(i);
    status |= v<< OpenHRP::RobotHardwareService::SERVO_STATE_SHIFT;
    v = robot->readServoAlarm(i);
    status |= v<< OpenHRP::RobotHardwareService::SERVO_ALARM_SHIFT;
    v = robot->readDriverTemperature(i);
    status |= v<< OpenHRP::RobotHardwareService::DRIVER_TEMP_SHIFT;
    rstate.servoState[i][0] = status;
    robot->readExtraServoState(i, (int *)(rstate.servoState[i].get_buffer()+1));
  }

  rstate.rateGyro.length(robot->numSensors(Sensor::RATE_GYRO));
  for (unsigned int i=0; i<rstate.rateGyro.length(); i++){
    rstate.rateGyro[i].length(3);
    robot->readGyroSensor(i, rstate.rateGyro[i].get_buffer());
  }

  rstate.accel.length(robot->numSensors(Sensor::ACCELERATION));
  for (unsigned int i=0; i<rstate.accel.length(); i++){
    rstate.accel[i].length(3);
    robot->readAccelerometer(i, rstate.accel[i].get_buffer());
  }

  rstate.force.length(robot->numSensors(Sensor::FORCE));
  for (unsigned int i=0; i<rstate.force.length(); i++){
    rstate.force[i].length(6);
    robot->readForceSensor(i, rstate.force[i].get_buffer());
  }

  robot->readPowerStatus(rstate.voltage, rstate.current);
}
 
void RobotHardware::getStatus2(OpenHRP::RobotHardwareService::RobotState2 &rstate2)
{
  getStatus(m_robot, rstate2);
#if defined(ROBOT_IOB_VERSION) && ROBOT_IOB_VERSION >= 2
  rstate2.batteries.length(m_robot->numBatteries());
  for(unsigned int i=0; i<rstate2.batteries.length(); i++){
      m_robot->readBatteryState(i, 
                                rstate2.batteries[i].voltage,
                                rstate2.batteries[i].current,
                                rstate2.batteries[i].soc);
  }
  rstate2.temperature.length(m_robot->numThermometers());
  for (unsigned int i=0; i<rstate2.temperature.length(); i++){
      m_robot->readThermometer(i, rstate2.temperature[i]);
  }
#endif
}

/*
RTC::ReturnCode_t RobotHardware::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t RobotHardware::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t RobotHardware::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t RobotHardware::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t RobotHardware::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/



extern "C"
{

  void RobotHardwareInit(RTC::Manager* manager)
  {
    RTC::Properties profile(robothardware_spec);
    manager->registerFactory(profile,
                             RTC::Create<RobotHardware>,
                             RTC::Delete<RobotHardware>);
  }

};


