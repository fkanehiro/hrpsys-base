// -*- C++ -*-
/*!
 * @file  ForwardKinematics.cpp
 * @brief null component
 * $Date$
 *
 * $Id$
 */

#include <rtm/CorbaNaming.h>
#include "ForwardKinematics.h"

#include "hrpModel/Link.h"
#include "hrpModel/ModelLoaderUtil.h"

typedef coil::Guard<coil::Mutex> Guard;

// Module specification
// <rtc-template block="module_spec">
static const char* nullcomponent_spec[] =
  {
    "implementation_id", "ForwardKinematics",
    "type_name",         "ForwardKinematics",
    "description",       "forward kinematics component",
    "version",           HRPSYS_PACKAGE_VERSION,
    "vendor",            "AIST",
    "category",          "example",
    "activity_type",     "DataFlowComponent",
    "max_instance",      "10",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    "conf.default.sensorAttachedLink", "",

    ""
  };
// </rtc-template>

ForwardKinematics::ForwardKinematics(RTC::Manager* manager)
  : RTC::DataFlowComponentBase(manager),
    // <rtc-template block="initializer">
    m_qIn("q", m_q),
    m_sensorRpyIn("sensorRpy", m_sensorRpy),
    m_qRefIn("qRef", m_qRef),
    m_basePosRefIn("basePosRef", m_basePosRef),
    m_baseRpyRefIn("baseRpyRef", m_baseRpyRef),
    m_ForwardKinematicsServicePort("ForwardKinematicsService"),
    // </rtc-template>
    dummy(0)
{
}

ForwardKinematics::~ForwardKinematics()
{
}



RTC::ReturnCode_t ForwardKinematics::onInitialize()
{
  std::cerr << "[" << m_profile.instance_name << "] onInitialize()" << std::endl;
  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  coil::Properties& ref = getProperties();
  bindParameter("sensorAttachedLink", m_sensorAttachedLinkName, ref["conf.default.sensorAttachedLink"].c_str());
  
  // </rtc-template>

  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("q", m_qIn);
  addInPort("sensorRpy", m_sensorRpyIn);
  addInPort("qRef", m_qRefIn);
  addInPort("basePosRef", m_basePosRefIn);
  addInPort("baseRpyRef", m_baseRpyRefIn);

  // Set OutPort buffer
  
  // Set service provider to Ports
  m_ForwardKinematicsServicePort.registerProvider("service0", "ForwardKinematicsService", m_service0);
  addPort(m_ForwardKinematicsServicePort);
  m_service0.setComp(this);
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  
  // </rtc-template>

  RTC::Properties& prop = getProperties();

  RTC::Manager& rtcManager = RTC::Manager::instance();
  std::string nameServer = rtcManager.getConfig()["corba.nameservers"];
  int comPos = nameServer.find(",");
  if (comPos < 0){
      comPos = nameServer.length();
  }
  nameServer = nameServer.substr(0, comPos);
  RTC::CorbaNaming naming(rtcManager.getORB(), nameServer.c_str());
  m_refBody = hrp::BodyPtr(new hrp::Body());
  if (!loadBodyFromModelLoader(m_refBody, prop["model"].c_str(), 
                               CosNaming::NamingContext::_duplicate(naming.getRootContext()))){
    std::cerr << "[" << m_profile.instance_name << "] failed to load model[" << prop["model"] << "]" << std::endl;
    return RTC::RTC_ERROR;
  }
  m_actBody = hrp::BodyPtr(new hrp::Body());
  if (!loadBodyFromModelLoader(m_actBody, prop["model"].c_str(), 
                               CosNaming::NamingContext::_duplicate(naming.getRootContext()))){
    std::cerr << "[" << m_profile.instance_name << "] failed to load model[" << prop["model"] << "]" << std::endl;
    return RTC::RTC_ERROR;
  }

  m_refLink = m_refBody->rootLink();
  m_actLink = m_actBody->rootLink();

  return RTC::RTC_OK;
}



/*
RTC::ReturnCode_t ForwardKinematics::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t ForwardKinematics::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t ForwardKinematics::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

RTC::ReturnCode_t ForwardKinematics::onActivated(RTC::UniqueId ec_id)
{
  std::cerr << "[" << m_profile.instance_name<< "] onActivated(" << ec_id << ")" << std::endl;
  if (m_sensorAttachedLinkName == ""){
    m_sensorAttachedLink = NULL;
  }else{
    m_sensorAttachedLink = m_actBody->link(m_sensorAttachedLinkName);
    if (!m_sensorAttachedLink){
      std::cerr << "can't find a link named " << m_sensorAttachedLinkName 
		<< std::endl;
      return RTC::RTC_ERROR;
    }
  }
  return RTC::RTC_OK;
}

RTC::ReturnCode_t ForwardKinematics::onDeactivated(RTC::UniqueId ec_id)
{
  std::cerr << "[" << m_profile.instance_name<< "] onDeactivated(" << ec_id << ")" << std::endl;
  return RTC::RTC_OK;
}

RTC::ReturnCode_t ForwardKinematics::onExecute(RTC::UniqueId ec_id)
{
  //std::cout << m_profile.instance_name<< ": onExecute(" << ec_id << ")" << std::endl;

  coil::TimeValue tm(coil::gettimeofday());
  m_tm.sec  = tm.sec();
  m_tm.nsec = tm.usec() * 1000;

  if (m_qIn.isNew()) {
      m_qIn.read();
      for (unsigned int i=0; i<m_actBody->numJoints(); i++){
          m_actBody->joint(i)->q = m_q.data[i];
      }
  }

  if (m_sensorRpyIn.isNew()) {
      m_sensorRpyIn.read();
      hrp::Matrix33 sensorR = hrp::rotFromRpy(m_sensorRpy.data.r,
                                              m_sensorRpy.data.p,
                                              m_sensorRpy.data.y);
      if (m_sensorAttachedLink){
        hrp::Matrix33 sensor2base(m_sensorAttachedLink->R.transpose()*m_actBody->rootLink()->R);
	hrp::Matrix33 baseR(sensorR*sensor2base);
	// to prevent numerical error
	hrp::Vector3 baseRpy = hrp::rpyFromRot(baseR);
        // use reference yaw angle instead of estimated one
        baseRpy[2] = m_baseRpyRef.data.y;
	m_actBody->rootLink()->R = hrp::rotFromRpy(baseRpy);
      }else{
	m_actBody->rootLink()->R = sensorR;
      }
  }

  if (m_qRefIn.isNew()) {
      m_qRefIn.read();
      for (unsigned int i=0; i<m_refBody->numJoints(); i++){
          m_refBody->joint(i)->q = m_qRef.data[i];
      }
  }

  if (m_basePosRefIn.isNew()){
      m_basePosRefIn.read();
      hrp::Link *root = m_refBody->rootLink();
      root->p[0] = m_basePosRef.data.x;
      root->p[1] = m_basePosRef.data.y;
      root->p[2] = m_basePosRef.data.z;
  }

  if (m_baseRpyRefIn.isNew()){
      m_baseRpyRefIn.read();
      hrp::Vector3 rpy;
      rpy[0] = m_baseRpyRef.data.r;
      rpy[1] = m_baseRpyRef.data.p;
      rpy[2] = m_baseRpyRef.data.y;
      m_refBody->rootLink()->R = hrp::rotFromRpy(rpy);

  }

  {
      Guard guard(m_bodyMutex);
      m_refBody->calcForwardKinematics();
      m_actBody->calcForwardKinematics();
  }

  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t ForwardKinematics::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t ForwardKinematics::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t ForwardKinematics::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t ForwardKinematics::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t ForwardKinematics::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

::CORBA::Boolean ForwardKinematics::getReferencePose(const char* linkname, RTC::TimedDoubleSeq_out pose,const char* frame_name)
{
    pose = new RTC::TimedDoubleSeq();
    Guard guard(m_bodyMutex);
    hrp::Link *l = m_refBody->link(linkname);
    if (!l) return false;
    hrp::Link *f = NULL;
    if (frame_name) {
        f = m_refBody->link(frame_name);
        if (!f) {
            std::cerr << "[getReferencePose] ERROR Could not find frame_name = " << frame_name << std::endl;
            return false;
        }
    }
    std::cerr << "[getReferencePose] linkaname = " << linkname << ", frame_name = " << (frame_name?frame_name:"(null)") << std::endl;
    hrp::Vector3 p = l->p;
    hrp::Matrix33 R = l->attitude();
    if (!!f) {
        p = f->attitude().transpose() * ( p - f->p );
        R = f->attitude().transpose() * R;
    }
    pose->tm = m_tm;
    pose->data.length(16);
    pose->data[ 0]=R(0,0);pose->data[ 1]=R(0,1);pose->data[ 2]=R(0,2);pose->data[ 3]=p[0];
    pose->data[ 4]=R(1,0);pose->data[ 5]=R(1,1);pose->data[ 6]=R(1,2);pose->data[ 7]=p[1];
    pose->data[ 8]=R(2,0);pose->data[ 9]=R(2,1);pose->data[10]=R(2,2);pose->data[11]=p[2];
    pose->data[12]=0;     pose->data[13]=0;     pose->data[14]=0;     pose->data[15]=1;
    return true;
}

::CORBA::Boolean ForwardKinematics::getCurrentPose(const char* linkname, RTC::TimedDoubleSeq_out pose, const char* frame_name)
{
    pose = new RTC::TimedDoubleSeq();
    Guard guard(m_bodyMutex);
    hrp::Link *l = m_actBody->link(linkname);
    if (!l) return false;
    hrp::Link *f = NULL;
    if (frame_name) {
        f = m_actBody->link(frame_name);
        if (!f) {
            std::cerr << "[getCurrentPose] ERROR Could not find frame_name = " << frame_name << std::endl;
            return false;
        }
    }
    std::cerr << "[getCurrentPose] linkaname = " << linkname << ", frame_name = " << (frame_name?frame_name:"(null)") << std::endl;
    hrp::Vector3 dp(m_refLink->p - m_actLink->p);

    hrp::Vector3 p(l->p + dp);
    hrp::Matrix33 R = l->attitude();
    if (!!f) {
        p = f->attitude().transpose() * ( p - f->p);
        R = f->attitude().transpose() * R;
    }
    pose->tm = m_tm;
    pose->data.length(16);
    pose->data[ 0]=R(0,0);pose->data[ 1]=R(0,1);pose->data[ 2]=R(0,2);pose->data[ 3]=p[0];
    pose->data[ 4]=R(1,0);pose->data[ 5]=R(1,1);pose->data[ 6]=R(1,2);pose->data[ 7]=p[1];
    pose->data[ 8]=R(2,0);pose->data[ 9]=R(2,1);pose->data[10]=R(2,2);pose->data[11]=p[2];
    pose->data[12]=0;     pose->data[13]=0;     pose->data[14]=0;     pose->data[15]=1;
    return true;
}

::CORBA::Boolean ForwardKinematics::getRelativeCurrentPosition(const char* linknameFrom, const char* linknameTo, const OpenHRP::ForwardKinematicsService::position target, OpenHRP::ForwardKinematicsService::position result)
{
    Guard guard(m_bodyMutex);
    hrp::Link *from = m_actBody->link(linknameFrom);
    hrp::Link *to = m_actBody->link(linknameTo);
    if (!from || !to) return false;
    hrp::Vector3 targetPrel(target[0], target[1], target[2]);
    hrp::Vector3 targetPabs(to->p+to->attitude()*targetPrel);
    hrp::Matrix33 Rt(from->attitude().transpose());
    hrp::Vector3 p(Rt*(targetPabs - from->p));
    result[ 0]=p(0);result[ 1]=p(1);result[ 2]=p(2);
    return true;
}

::CORBA::Boolean ForwardKinematics::selectBaseLink(const char* linkname)
{
    Guard guard(m_bodyMutex);
    hrp::Link *l = m_refBody->link(linkname);
    if (!l) return false;
    m_refLink = l;
    m_actLink = m_actBody->link(linkname);
    return true;
}

extern "C"
{

  void ForwardKinematicsInit(RTC::Manager* manager)
  {
    RTC::Properties profile(nullcomponent_spec);
    manager->registerFactory(profile,
                             RTC::Create<ForwardKinematics>,
                             RTC::Delete<ForwardKinematics>);
  }

};


