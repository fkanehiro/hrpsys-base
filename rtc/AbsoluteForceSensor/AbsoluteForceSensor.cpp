// -*- C++ -*-
/*!
 * @file  AbsoluteForceSensor.cpp
 * @brief virtual force sensor component
 * $Date$
 *
 * $Id$
 */

#include "AbsoluteForceSensor.h"
#include <rtm/CorbaNaming.h>
#include <hrpModel/ModelLoaderUtil.h>
#include <hrpUtil/MatrixSolvers.h>
#include <hrpModel/Sensor.h>

// Module specification
// <rtc-template block="module_spec">
static const char* absoluteforcesensor_spec[] =
  {
    "implementation_id", "AbsoluteForceSensor",
    "type_name",         "AbsoluteForceSensor",
    "description",       "null component",
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

AbsoluteForceSensor::AbsoluteForceSensor(RTC::Manager* manager)
  : RTC::DataFlowComponentBase(manager),
    // <rtc-template block="initializer">
    m_qCurrentIn("qCurrent", m_qCurrent),
    m_rpyIn("rpy", m_rpy),
    m_AbsoluteForceSensorServicePort("AbsoluteForceSensorService"),
    // </rtc-template>
    m_debugLevel(0)
{
  m_service0.absfsensor(this);
}

AbsoluteForceSensor::~AbsoluteForceSensor()
{
}



RTC::ReturnCode_t AbsoluteForceSensor::onInitialize()
{
  std::cout << m_profile.instance_name << ": onInitialize()" << std::endl;
  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  bindParameter("debugLevel", m_debugLevel, "0");
  
  // </rtc-template>

  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("qCurrent", m_qCurrentIn);
  addInPort("rpy", m_rpyIn);

  // Set OutPort buffer
  
  // Set service provider to Ports
  m_AbsoluteForceSensorServicePort.registerProvider("service0", "AbsoluteForceSensorService", m_service0);
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  addPort(m_AbsoluteForceSensorServicePort);
  
  // </rtc-template>

  RTC::Properties& prop = getProperties();
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
  }

  int nforce = m_robot->numSensors(hrp::Sensor::FORCE);
  m_force.resize(nforce);
  m_forceOut.resize(nforce);
  m_forceIn.resize(nforce);
  for (size_t i = 0; i < nforce; i++) {
    hrp::Sensor *s = m_robot->sensor(hrp::Sensor::FORCE, i);
    m_forceOut[i] = new OutPort<TimedDoubleSeq>(std::string("off_"+s->name).c_str(), m_force[i]);
    m_forceIn[i] = new InPort<TimedDoubleSeq>(s->name.c_str(), m_force[i]);
    m_force[i].data.length(6);
    registerInPort(s->name.c_str(), *m_forceIn[i]);
    registerOutPort(std::string("off_"+s->name).c_str(), *m_forceOut[i]);
    m_forcemoment_offset_param.insert(std::pair<std::string, ForceMomentOffsetParam>(s->name, ForceMomentOffsetParam()));
  }
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t AbsoluteForceSensor::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t AbsoluteForceSensor::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t AbsoluteForceSensor::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

RTC::ReturnCode_t AbsoluteForceSensor::onActivated(RTC::UniqueId ec_id)
{
  std::cout << m_profile.instance_name<< ": onActivated(" << ec_id << ")" << std::endl;
  return RTC::RTC_OK;
}

RTC::ReturnCode_t AbsoluteForceSensor::onDeactivated(RTC::UniqueId ec_id)
{
  std::cout << m_profile.instance_name<< ": onDeactivated(" << ec_id << ")" << std::endl;
  return RTC::RTC_OK;
}

#define DEBUGP ((m_debugLevel==1 && loop%200==0) || m_debugLevel > 1 )
RTC::ReturnCode_t AbsoluteForceSensor::onExecute(RTC::UniqueId ec_id)
{
  //std::cout << m_profile.instance_name<< ": onExecute(" << ec_id << ")" << std::endl;
  static int loop = 0;
  loop ++;
  for (unsigned int i=0; i<m_forceIn.size(); i++){
    if ( m_forceIn[i]->isNew() ) {
      m_forceIn[i]->read();
    }
  }
  hrp::Vector3 rpy;
  if (m_rpyIn.isNew()) {
    m_rpyIn.read();
    rpy = hrp::Vector3(m_rpy.data.r, m_rpy.data.p, m_rpy.data.y);
  } else {
    rpy = hrp::Vector3::Zero();
  }
  if (m_qCurrentIn.isNew()) {
    m_qCurrentIn.read();
    for ( int i = 0; i < m_robot->numJoints(); i++ ){
      m_robot->joint(i)->q = m_qCurrent.data[i];
    }
    //
    updateRootLinkPosRot(rpy);
    m_robot->calcForwardKinematics();
    for (unsigned int i=0; i<m_forceIn.size(); i++){
      if ( m_force[i].data.length()==6 ) {
        std::string sensor_name = m_forceIn[i]->name();
        hrp::ForceSensor* sensor = m_robot->sensor<hrp::ForceSensor>(sensor_name);
        hrp::Vector3 data_p(m_force[i].data[0], m_force[i].data[1], m_force[i].data[2]);
        hrp::Vector3 data_r(m_force[i].data[3], m_force[i].data[4], m_force[i].data[5]);
        if ( DEBUGP ) {
          std::cerr << "forces[" << m_forceIn[i]->name() << "]" << std::endl;;
          std::cerr << "raw force : " << data_p[0] << " " << data_p[1] << " " << data_p[2] << std::endl;
          std::cerr << "raw moment : " << data_r[0] << " " << data_r[1] << " " << data_r[2] << std::endl;
        }
        if ( sensor ) {
          // real force sensor
          hrp::Matrix33 sensorR = sensor->link->R * sensor->localR;
          hrp::Vector3 mg = hrp::Vector3(0,0, m_forcemoment_offset_param[sensor_name].link_offset_mass * grav * -1);
          // force and moments which do not include offsets
          hrp::Vector3 off_force = sensorR * (data_p - m_forcemoment_offset_param[sensor_name].force_offset) - mg;
          hrp::Vector3 off_moment = sensorR * (data_r - m_forcemoment_offset_param[sensor_name].moment_offset) - hrp::Vector3(sensorR * m_forcemoment_offset_param[sensor->name].link_offset_centroid).cross(mg);
          // convert absolute force -> sensor local force
          off_force = hrp::Vector3(sensorR.transpose() * off_force);
          off_moment = hrp::Vector3(sensorR.transpose() * off_moment);
          for (size_t j = 0; j < 3; j++) {
            m_force[i].data[j] = off_force(j);
            m_force[i].data[3+j] = off_moment(j);
          }
          if ( DEBUGP ) {
            std::cerr << "off force : " << off_force[0] << " " << off_force[1] << " " << off_force[2] << std::endl;
            std::cerr << "off moment : " << off_moment[0] << " " << off_moment[1] << " " << off_moment[2] << std::endl;
          }
        } else {
          std::cerr << "unknwon force param" << std::endl;
        }
      }
    }
  }
  for (unsigned int i=0; i<m_forceOut.size(); i++){
    m_forceOut[i]->write();
  }
  return RTC::RTC_OK;
}

void AbsoluteForceSensor::updateRootLinkPosRot (const hrp::Vector3& rpy)
{
  if ( m_robot->numSensors(hrp::Sensor::ACCELERATION) > 0) {
    hrp::Sensor *sensor = m_robot->sensor(hrp::Sensor::ACCELERATION, 0);
    hrp::Matrix33 tmpr;
    rats::rotm3times(tmpr, hrp::Matrix33(sensor->link->R*sensor->localR).transpose(), m_robot->rootLink()->R);
    rats::rotm3times(m_robot->rootLink()->R, hrp::rotFromRpy(rpy(0), rpy(1), rpy(2)), tmpr);
  }
}

bool AbsoluteForceSensor::setForceMomentOffsetParam(const std::string& i_name_, const AbsoluteForceSensorService::forcemomentOffsetParam& i_param_)
{
  if (m_forcemoment_offset_param.find(i_name_) != m_forcemoment_offset_param.end()) {
    // std::cerr << "OK " << i_name_ << " in setForceMomentOffsetParam" << std::endl;
    memcpy(m_forcemoment_offset_param[i_name_].force_offset.data(), i_param_.force_offset.get_buffer(), sizeof(double) * 3);
    memcpy(m_forcemoment_offset_param[i_name_].moment_offset.data(), i_param_.moment_offset.get_buffer(), sizeof(double) * 3);
    memcpy(m_forcemoment_offset_param[i_name_].link_offset_centroid.data(), i_param_.link_offset_centroid.get_buffer(), sizeof(double) * 3);
    m_forcemoment_offset_param[i_name_].link_offset_mass = i_param_.link_offset_mass;
  } else {
    std::cerr << "No such limb " << i_name_ << " in setForceMomentOffsetParam" << std::endl;
  }
  return true;
}

bool AbsoluteForceSensor::getForceMomentOffsetParam(const std::string& i_name_, AbsoluteForceSensorService::forcemomentOffsetParam& i_param_)
{
  if (m_forcemoment_offset_param.find(i_name_) != m_forcemoment_offset_param.end()) {
    // std::cerr << "OK " << i_name_ << " in getForceMomentOffsetParam" << std::endl;
    memcpy(i_param_.force_offset.get_buffer(), m_forcemoment_offset_param[i_name_].force_offset.data(), sizeof(double) * 3);
    memcpy(i_param_.moment_offset.get_buffer(), m_forcemoment_offset_param[i_name_].force_offset.data(), sizeof(double) * 3);
    memcpy(i_param_.link_offset_centroid.get_buffer(), m_forcemoment_offset_param[i_name_].link_offset_centroid.data(), sizeof(double) * 3);
    i_param_.link_offset_mass = m_forcemoment_offset_param[i_name_].link_offset_mass;
  } else {
    std::cerr << "No such limb " << i_name_ << " in getForceMomentOffsetParam" << std::endl;
  }
  return true;
}

/*
RTC::ReturnCode_t AbsoluteForceSensor::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t AbsoluteForceSensor::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t AbsoluteForceSensor::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t AbsoluteForceSensor::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t AbsoluteForceSensor::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

extern "C"
{

  void AbsoluteForceSensorInit(RTC::Manager* manager)
  {
    RTC::Properties profile(absoluteforcesensor_spec);
    manager->registerFactory(profile,
                             RTC::Create<AbsoluteForceSensor>,
                             RTC::Delete<AbsoluteForceSensor>);
  }

};


