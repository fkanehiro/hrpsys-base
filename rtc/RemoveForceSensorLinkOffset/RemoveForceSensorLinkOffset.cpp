// -*- C++ -*-
/*!
 * @file  RemoveForceSensorLinkOffset.cpp
 * @brief virtual force sensor component
 * $Date$
 *
 * $Id$
 */

#include "RemoveForceSensorLinkOffset.h"
#include <rtm/CorbaNaming.h>
#include <hrpModel/ModelLoaderUtil.h>
#include <hrpUtil/MatrixSolvers.h>
#include <hrpModel/Sensor.h>

// Module specification
// <rtc-template block="module_spec">
static const char* removeforcesensorlinkoffset_spec[] =
  {
    "implementation_id", "RemoveForceSensorLinkOffset",
    "type_name",         "RemoveForceSensorLinkOffset",
    "description",       "null component",
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

RemoveForceSensorLinkOffset::RemoveForceSensorLinkOffset(RTC::Manager* manager)
  : RTC::DataFlowComponentBase(manager),
    // <rtc-template block="initializer">
    m_qCurrentIn("qCurrent", m_qCurrent),
    m_rpyIn("rpy", m_rpy),
    m_RemoveForceSensorLinkOffsetServicePort("RemoveForceSensorLinkOffsetService"),
    // </rtc-template>
    m_debugLevel(0)
{
  m_service0.rmfsoff(this);
}

RemoveForceSensorLinkOffset::~RemoveForceSensorLinkOffset()
{
}



RTC::ReturnCode_t RemoveForceSensorLinkOffset::onInitialize()
{
  std::cerr << "[" << m_profile.instance_name << "] onInitialize()" << std::endl;
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
  m_RemoveForceSensorLinkOffsetServicePort.registerProvider("service0", "RemoveForceSensorLinkOffsetService", m_service0);
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  addPort(m_RemoveForceSensorLinkOffsetServicePort);
  
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
      std::cerr << "[" << m_profile.instance_name << "] failed to load model[" << prop["model"] << "]" << std::endl;
      return RTC::RTC_ERROR;
  }

  unsigned int nforce = m_robot->numSensors(hrp::Sensor::FORCE);
  m_force.resize(nforce);
  m_forceOut.resize(nforce);
  m_forceIn.resize(nforce);
  for (unsigned int i = 0; i < nforce; i++) {
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
RTC::ReturnCode_t RemoveForceSensorLinkOffset::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t RemoveForceSensorLinkOffset::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t RemoveForceSensorLinkOffset::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

RTC::ReturnCode_t RemoveForceSensorLinkOffset::onActivated(RTC::UniqueId ec_id)
{
  std::cerr << "[" << m_profile.instance_name<< "] onActivated(" << ec_id << ")" << std::endl;
  return RTC::RTC_OK;
}

RTC::ReturnCode_t RemoveForceSensorLinkOffset::onDeactivated(RTC::UniqueId ec_id)
{
  std::cerr << "[" << m_profile.instance_name<< "] onDeactivated(" << ec_id << ")" << std::endl;
  return RTC::RTC_OK;
}

#define DEBUGP ((m_debugLevel==1 && loop%200==0) || m_debugLevel > 1 )
RTC::ReturnCode_t RemoveForceSensorLinkOffset::onExecute(RTC::UniqueId ec_id)
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
    for ( unsigned int i = 0; i < m_robot->numJoints(); i++ ){
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
          std::cerr << "[" << m_profile.instance_name << "] wrench [" << m_forceIn[i]->name() << "]" << std::endl;;
          std::cerr << "[" << m_profile.instance_name << "]   raw force = " << data_p.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << std::endl;
          std::cerr << "[" << m_profile.instance_name << "]   raw moment = " << data_r.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << std::endl;
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
            std::cerr << "[" << m_profile.instance_name << "]   off force = " << off_force.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << std::endl;
            std::cerr << "[" << m_profile.instance_name << "]   off moment = " << off_moment.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << std::endl;
          }
        } else {
          std::cerr << "[" << m_profile.instance_name << "] unknwon force param " << sensor_name << std::endl;
        }
      }
    }
  }
  for (unsigned int i=0; i<m_forceOut.size(); i++){
    m_forceOut[i]->write();
  }
  return RTC::RTC_OK;
}

void RemoveForceSensorLinkOffset::updateRootLinkPosRot (const hrp::Vector3& rpy)
{
  if ( m_robot->numSensors(hrp::Sensor::ACCELERATION) > 0) {
    hrp::Sensor *sensor = m_robot->sensor(hrp::Sensor::ACCELERATION, 0);
    hrp::Matrix33 tmpr;
    rats::rotm3times(tmpr, hrp::Matrix33(sensor->link->R*sensor->localR).transpose(), m_robot->rootLink()->R);
    rats::rotm3times(m_robot->rootLink()->R, hrp::rotFromRpy(rpy(0), rpy(1), rpy(2)), tmpr);
  }
}

void RemoveForceSensorLinkOffset::printForceMomentOffsetParam(const std::string& i_name_)
{
  std::cerr << "[" << m_profile.instance_name << "]   force_offset = " << m_forcemoment_offset_param[i_name_].force_offset.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[N]" << std::endl;
  std::cerr << "[" << m_profile.instance_name << "]   moment_offset = " << m_forcemoment_offset_param[i_name_].moment_offset.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[Nm]" << std::endl;
  std::cerr << "[" << m_profile.instance_name << "]   link_offset_centroid = " << m_forcemoment_offset_param[i_name_].link_offset_centroid.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[m]" << std::endl;
  std::cerr << "[" << m_profile.instance_name << "]   link_offset_mass = " << m_forcemoment_offset_param[i_name_].link_offset_mass << "[kg]" << std::endl;
};

bool RemoveForceSensorLinkOffset::setForceMomentOffsetParam(const std::string& i_name_, const RemoveForceSensorLinkOffsetService::forcemomentOffsetParam& i_param_)
{
  std::cerr << "[" << m_profile.instance_name << "] setForceMomentOffsetParam [" << i_name_ << "]" << std::endl;
  if (m_forcemoment_offset_param.find(i_name_) != m_forcemoment_offset_param.end()) {
    memcpy(m_forcemoment_offset_param[i_name_].force_offset.data(), i_param_.force_offset.get_buffer(), sizeof(double) * 3);
    memcpy(m_forcemoment_offset_param[i_name_].moment_offset.data(), i_param_.moment_offset.get_buffer(), sizeof(double) * 3);
    memcpy(m_forcemoment_offset_param[i_name_].link_offset_centroid.data(), i_param_.link_offset_centroid.get_buffer(), sizeof(double) * 3);
    m_forcemoment_offset_param[i_name_].link_offset_mass = i_param_.link_offset_mass;
    printForceMomentOffsetParam(i_name_);
    return true;
  } else {
    std::cerr << "[" << m_profile.instance_name << "]   No such limb"<< std::endl;
    return false;
  }
}

bool RemoveForceSensorLinkOffset::getForceMomentOffsetParam(const std::string& i_name_, RemoveForceSensorLinkOffsetService::forcemomentOffsetParam& i_param_)
{
  if (m_forcemoment_offset_param.find(i_name_) != m_forcemoment_offset_param.end()) {
    // std::cerr << "OK " << i_name_ << " in getForceMomentOffsetParam" << std::endl;
    memcpy(i_param_.force_offset.get_buffer(), m_forcemoment_offset_param[i_name_].force_offset.data(), sizeof(double) * 3);
    memcpy(i_param_.moment_offset.get_buffer(), m_forcemoment_offset_param[i_name_].moment_offset.data(), sizeof(double) * 3);
    memcpy(i_param_.link_offset_centroid.get_buffer(), m_forcemoment_offset_param[i_name_].link_offset_centroid.data(), sizeof(double) * 3);
    i_param_.link_offset_mass = m_forcemoment_offset_param[i_name_].link_offset_mass;
    return true;
  } else {
    std::cerr << "[" << m_profile.instance_name << "] No such limb " << i_name_ << " in getForceMomentOffsetParam" << std::endl;
    return false;
  }
}

bool RemoveForceSensorLinkOffset::loadForceMomentOffsetParams(const std::string& filename)
{
  std::cerr << "[" << m_profile.instance_name << "] loadForceMomentOffsetParams" << std::endl;
  std::ifstream ifs(filename.c_str());
  if (ifs.is_open()){
    while(ifs.eof()==0){
      std::string tmps;
      ForceMomentOffsetParam tmpp;
      if ( ifs >> tmps ) {
          if ( m_forcemoment_offset_param.find(tmps) != m_forcemoment_offset_param.end()) {
              for (size_t i = 0; i < 3; i++) ifs >> tmpp.force_offset(i);
              for (size_t i = 0; i < 3; i++) ifs >> tmpp.moment_offset(i);
              for (size_t i = 0; i < 3; i++) ifs >> tmpp.link_offset_centroid(i);
              ifs >> tmpp.link_offset_mass;
              m_forcemoment_offset_param[tmps] = tmpp;
              std::cerr << "[" << m_profile.instance_name << "]   " << tmps << "" << std::endl;
              printForceMomentOffsetParam(tmps);
          } else {
              std::cerr << "[" << m_profile.instance_name << "] no such (" << tmps << ")" << std::endl;
              return false;
          }
      }
    }
  } else {
    std::cerr << "[" << m_profile.instance_name << "] failed to open(" << filename << ")" << std::endl;
    return false;
  }
  return true;
};

bool RemoveForceSensorLinkOffset::dumpForceMomentOffsetParams(const std::string& filename)
{
  std::cerr << "[" << m_profile.instance_name << "] dumpForceMomentOffsetParams" << std::endl;
  std::ofstream ofs(filename.c_str());
  if (ofs.is_open()){
    for ( std::map<std::string, ForceMomentOffsetParam>::iterator it = m_forcemoment_offset_param.begin(); it != m_forcemoment_offset_param.end(); it++ ) {
      ofs << it->first << " ";
      ofs << it->second.force_offset[0] << " " << it->second.force_offset[1] << " " << it->second.force_offset[2] << " ";
      ofs << it->second.moment_offset[0] << " " << it->second.moment_offset[1] << " " << it->second.moment_offset[2] << " ";
      ofs << it->second.link_offset_centroid[0] << " " << it->second.link_offset_centroid[1] << " " << it->second.link_offset_centroid[2] << " ";
      ofs << it->second.link_offset_mass << std::endl;
    }
  } else {
    std::cerr << "[" << m_profile.instance_name << "] failed to open(" << filename << ")" << std::endl;
    return false;
  }
  return true;
};

/*
RTC::ReturnCode_t RemoveForceSensorLinkOffset::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t RemoveForceSensorLinkOffset::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t RemoveForceSensorLinkOffset::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t RemoveForceSensorLinkOffset::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t RemoveForceSensorLinkOffset::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

extern "C"
{

  void RemoveForceSensorLinkOffsetInit(RTC::Manager* manager)
  {
    RTC::Properties profile(removeforcesensorlinkoffset_spec);
    manager->registerFactory(profile,
                             RTC::Create<RemoveForceSensorLinkOffset>,
                             RTC::Delete<RemoveForceSensorLinkOffset>);
  }

};


