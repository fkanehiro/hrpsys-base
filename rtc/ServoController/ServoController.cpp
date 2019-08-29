// -*- C++ -*-
/*!
 * @file  ServoController.cpp
 * @brief servo controller component
 * $Date$
 *
 * $Id$
 */

#include <rtm/CorbaNaming.h>
#include <hrpModel/Link.h>
#include <hrpModel/ModelLoaderUtil.h>
#include "ServoController.h"
#include "hrpsys/util/VectorConvert.h"

#include "ServoSerial.h"

using namespace std;

// Module specification
// <rtc-template block="module_spec">
static const char* nullcomponent_spec[] =
  {
    "implementation_id", "ServoController",
    "type_name",         "ServoController",
    "description",       "null component",
    "version",           HRPSYS_PACKAGE_VERSION,
    "vendor",            "AIST",
    "category",          "example",
    "activity_type",     "DataFlowComponent",
    "max_instance",      "10",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    ""
  };
// </rtc-template>

ServoController::ServoController(RTC::Manager* manager)
  : RTC::DataFlowComponentBase(manager),
    // <rtc-template block="initializer">
    m_ServoControllerServicePort("ServoControllerService")
    // </rtc-template>
{
    m_service0.servo(this);
}

ServoController::~ServoController()
{
}



RTC::ReturnCode_t ServoController::onInitialize()
{
  std::cout << m_profile.instance_name << ": onInitialize()" << std::endl;
  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  
  // </rtc-template>

  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers

  // Set OutPort buffer
  
  // Set service provider to Ports
  m_ServoControllerServicePort.registerProvider("service0", "ServoControllerService", m_service0);
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  addPort(m_ServoControllerServicePort);
  
  // </rtc-template>

  RTC::Properties& prop = getProperties();

  // get servo.devname
  std::string devname = prop["servo.devname"];
  if ( devname  == "" ) {
      std::cerr << "\e[1;31m[WARNING] " <<  m_profile.instance_name << ": needs servo.devname property\e[0m" << std::endl;
      std::cerr << "\e[1;31m[WARNING] " <<  m_profile.instance_name << ": running in dummy mode\e[0m" << std::endl;
      return RTC::RTC_OK;
  }

  // get servo.id
  coil::vstring servo_ids = coil::split(prop["servo.id"], ",");
  if ( servo_ids.size() == 0 ) {
      std::cerr << "\e[1;31m[ERROR] " <<  m_profile.instance_name << ": needs servo.id property\e[0m" << std::endl;
      return RTC::RTC_ERROR;
  }
  servo_id.resize(servo_ids.size());
  for(unsigned int i = 0; i < servo_ids.size(); i++) {
      coil::stringTo(servo_id[i], servo_ids[i].c_str());
  }

  std::cout << m_profile.instance_name << ": servo_id : ";
  for(unsigned int i = 0; i < servo_id.size(); i++) {
      std::cerr << servo_id[i] << " ";
  }
  std::cerr << std::endl;

  // get servo.offset
  coil::vstring servo_offsets = coil::split(prop["servo.offset"], ",");
  if ( servo_offsets.size() == 0 ) {
      servo_offset.resize(servo_ids.size());
  } else {
      servo_offset.resize(servo_offsets.size());
  }
  if ( servo_ids.size() != servo_offset.size() ) {
      std::cerr << "\e[1;31m[ERROR] " <<  m_profile.instance_name << ": servo.id and servo.offset property must have same length\e[0m" << std::endl;
      return RTC::RTC_ERROR;
  }
  for(unsigned int i = 0; i < servo_offsets.size(); i++) {
      coil::stringTo(servo_offset[i], servo_offsets[i].c_str());
  }

  std::cout << m_profile.instance_name << ": servo_offset : ";
  for(unsigned int i = 0; i < servo_offset.size(); i++) {
      std::cerr << servo_offset[i] << " ";
  }
  std::cerr << std::endl;

 // get servo.dir
  coil::vstring servo_dirs = coil::split(prop["servo.dir"], ",");
  if ( servo_dirs.size() == 0 ) {
      servo_dir.assign(servo_ids.size(), 1);
  } else {
      servo_dir.assign(servo_dirs.size(), 1);
  }
  if ( servo_ids.size() != servo_dir.size() ) {
      std::cerr << "\e[1;31m[ERROR] " <<  m_profile.instance_name << ": servo.id and servo.dir property must have same length\e[0m" << std::endl;
      return RTC::RTC_ERROR;
  }
  for(unsigned int i = 0; i < servo_dirs.size(); i++) {
      coil::stringTo(servo_dir[i], servo_dirs[i].c_str());
  }

  std::cout << m_profile.instance_name << ": servo_dir : ";
  for(unsigned int i = 0; i < servo_dir.size(); i++) {
      std::cerr << servo_dir[i] << " ";
  }
  std::cerr << std::endl;

  serial = new ServoSerial((char *)(devname.c_str()));

  return RTC::RTC_OK;
}



RTC::ReturnCode_t ServoController::onFinalize()
{
    if ( serial ) delete serial;
    return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t ServoController::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t ServoController::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

RTC::ReturnCode_t ServoController::onActivated(RTC::UniqueId ec_id)
{
  std::cout << m_profile.instance_name<< ": onActivated(" << ec_id << ")" << std::endl;
  if ( ! serial ) return RTC::RTC_OK;

  return RTC::RTC_OK;
}

RTC::ReturnCode_t ServoController::onDeactivated(RTC::UniqueId ec_id)
{
  std::cout << m_profile.instance_name<< ": onDeactivated(" << ec_id << ")" << std::endl;
  if ( ! serial ) return RTC::RTC_OK;

  return RTC::RTC_OK;
}

RTC::ReturnCode_t ServoController::onExecute(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t ServoController::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t ServoController::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t ServoController::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t ServoController::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t ServoController::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

bool ServoController::setJointAngle(short id, double angle, double tm)
{
    if ( ! serial ) return true;
    double rad = angle * M_PI / 180;
    for(unsigned int i=0; i<servo_id.size(); i++){
      if(servo_id[i]==id) serial->setPosition(id,rad+servo_offset[i], tm);
    }
    return true;
}

bool ServoController::setJointAngles(const OpenHRP::ServoControllerService::dSequence angles, double tm)
{
    if ( ! serial ) return true;

    int id[servo_id.size()];
    double tms[servo_id.size()];
    double rad[servo_id.size()];
    for( unsigned int i = 0; i < servo_id.size(); i++ ) {
        id[i] = servo_id[i];
        tms[i] = tm;
        rad[i] = (angles.get_buffer()[i]*servo_dir[i]+servo_offset[i]);
    }
    if ( angles.length() != servo_id.size() ) {
        std::cerr << "[ERROR] " <<  m_profile.instance_name << ": size of servo.id(" << angles.length() << ") is not correct, expected" << servo_id.size() << std::endl;
        return false;
    }
    serial->setPositions(servo_id.size(), id, rad, tms);
    return true;
}

bool ServoController::getJointAngle(short id, double &angle)
{
    if ( ! serial ) return true;

    int ret = serial->getPosition(id, &angle);
    for(unsigned int i=0; i<servo_id.size(); i++){
      if(servo_id[i]==id){
        double servo_offset_angle = servo_offset[i] * 180 / M_PI;
        angle -= servo_offset_angle;
      }
    }

    if (ret < 0) return false;
    return true;
}

bool ServoController::getJointAngles(OpenHRP::ServoControllerService::dSequence_out &angles)
{
    if ( ! serial ) return true;

    int ret;

    angles = new OpenHRP::ServoControllerService::dSequence();
    angles->length(servo_id.size());
    for(unsigned int i=0; i < servo_id.size(); i++){
        ret = serial->getPosition(servo_id[i], &(angles->get_buffer()[i]));
        if (ret < 0) return false;
    }
    return true;
}

bool ServoController::addJointGroup(const char *gname, const ::OpenHRP::ServoControllerService::iSequence ids)
{
    if ( ! serial ) return true;

    std::vector<int> indices;
    for (size_t i=0; i<ids.length(); i++){
        indices.push_back(ids[i]);
    }
    joint_groups[gname] = indices;

    return true;
}

bool ServoController::removeJointGroup(const char *gname)
{
    if ( ! serial ) return true;

    joint_groups.erase(gname);
}

bool ServoController::setJointAnglesOfGroup(const char *gname, const OpenHRP::ServoControllerService::dSequence angles, double tm)
{
    if ( ! serial ) return true;

    if ( joint_groups.find(gname) != joint_groups.end()) {
        unsigned int len = joint_groups[gname].size();
        if ( angles.length() != len ) {
            std::cerr << "[ERROR] " <<  m_profile.instance_name << ": size of servo.id(" << angles.length() << ") is not correct, expected" << len << std::endl;
            return false;
        }
        int id[len];
        double tms[len];
        double rad[len];
        for( unsigned int i = 0; i < len; i++ ) {
            id[i] = joint_groups[gname][i];
            tms[i] = tm;
            double offset, dir;
            for( unsigned int j = 0; j < servo_id.size(); j++ ) {
                if ( servo_id[j] == id[i]) {
                    offset = servo_offset[j];
                    dir = servo_dir[j];
                }
            }
            rad[i] = (angles.get_buffer()[i])*dir+offset;
        }
        serial->setPositions(servo_id.size(), id, rad, tms);
    }
    return true;
}

bool ServoController::setMaxTorque(short id, short percentage)
{
    if ( ! serial ) return true;

    int ret = serial->setMaxTorque(id, percentage);

    if (ret < 0) return false;
    return true;
}

bool ServoController::setReset(short id)
{
    if ( ! serial ) return true;

    int ret = serial->setReset(id);

    if (ret < 0) return false;
    return true;
}

bool ServoController::getDuration(short id, double &duration)
{
    if ( ! serial ) return true;

    int ret = serial->getDuration(id, &duration);

    if (ret < 0) return false;
    return true;
}

bool ServoController::getSpeed(short id, double &speed)
{
    if ( ! serial ) return true;

    int ret = serial->getSpeed(id, &speed);

    if (ret < 0) return false;
    return true;
}

bool ServoController::getMaxTorque(short id, short &percentage)
{
    if ( ! serial ) return true;

    int ret = serial->getMaxTorque(id, &percentage);

    if (ret < 0) return false;
    return true;
}

bool ServoController::getTorque(short id, double &torque)
{
    if ( ! serial ) return true;

    int ret = serial->getTorque(id, &torque);

    if (ret < 0) return false;
    return true;
}

bool ServoController::getTemperature(short id, double &temperature)
{
    if ( ! serial ) return true;

    int ret = serial->getTemperature(id, &temperature);

    if (ret < 0) return false;
    return true;
}

bool ServoController::getVoltage(short id, double &voltage)
{
    if ( ! serial ) return true;

    int ret = serial->getVoltage(id, &voltage);

    if (ret < 0) return false;
    return true;
}

bool ServoController::servoOn()
{
    if ( ! serial ) return true;

    int ret;

    for (vector<int>::iterator it = servo_id.begin(); it != servo_id.end(); it++ ){
        ret = serial->setTorqueOn(*it);
        if (ret < 0) return false;
    }

    return true;
}

bool ServoController::servoOff()
{
    if ( ! serial ) return true;

    int ret;

    for (vector<int>::iterator it = servo_id.begin(); it != servo_id.end(); it++ ){
        ret = serial->setTorqueOff(*it);
        if (ret < 0) return false;
    }
    return true;
}


extern "C"
{

  void ServoControllerInit(RTC::Manager* manager)
  {
    RTC::Properties profile(nullcomponent_spec);
    manager->registerFactory(profile,
                             RTC::Create<ServoController>,
                             RTC::Delete<ServoController>);
  }

};


