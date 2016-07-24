// -*- C++ -*-
/*!
 * @file  SelfCollisionChecker.cpp
 * @brief OpenNI grabber
 * $Date$
 *
 * $Id$
 */

#include <rtm/CorbaNaming.h>
#include <hrpModel/Link.h>
#include <hrpModel/ModelLoaderUtil.h>
#include "SelfCollisionChecker.h"

using namespace hrp;

// Module specification
// <rtc-template block="module_spec">
static const char* spec[] =
  {
    "implementation_id", "SelfCollisionChecker",
    "type_name",         "SelfCollisionChecker",
    "description",       "self collision checker",
    "version",           HRPSYS_PACKAGE_VERSION,
    "vendor",            "AIST",
    "category",          "example",
    "activity_type",     "DataFlowComponent",
    "max_instance",      "10",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    "conf.default.interval", "5",

    ""
  };
// </rtc-template>

SelfCollisionChecker::SelfCollisionChecker(RTC::Manager* manager)
  : RTC::DataFlowComponentBase(manager),
    // <rtc-template block="initializer">
    m_qIn("q", m_q),
    // </rtc-template>
    dummy(0)
{
}

SelfCollisionChecker::~SelfCollisionChecker()
{
}



RTC::ReturnCode_t SelfCollisionChecker::onInitialize()
{
  std::cout << m_profile.instance_name << ": onInitialize()" << std::endl;
  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  bindParameter("interval", m_interval, "5");
  
  // </rtc-template>

  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("q", m_qIn);

  // Set OutPort buffer
  
  // Set service provider to Ports
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  
  // </rtc-template>

  RTC::Properties& prop = getProperties();
  m_robot = hrp::BodyPtr(new Body());
  
  RTC::Manager& rtcManager = RTC::Manager::instance();
  std::string nameServer = rtcManager.getConfig()["corba.nameservers"];
  int comPos = nameServer.find(",");
  if (comPos < 0){
      comPos = nameServer.length();
  }
  nameServer = nameServer.substr(0, comPos);
  RTC::CorbaNaming naming(rtcManager.getORB(), nameServer.c_str());
  if (!loadBodyFromModelLoader(m_robot, prop["model"].c_str(), 
                               CosNaming::NamingContext::_duplicate(naming.getRootContext()), true)){
      std::cerr << "[" << m_profile.instance_name << "] Error: failed to load model[" << prop["model"] << "]" 
                << std::endl;
      return RTC::RTC_ERROR;
  }

  for (int i=0; i<m_robot->numLinks(); i++){
      Link *link1 = m_robot->link(i);
      for (int j=i+1; j<m_robot->numLinks(); j++){
          Link *link2 = m_robot->link(j);
          if (link1->parent != link2 && link2->parent != link1){
              m_checkPairs.push_back(ColdetModelPair(link1->coldetModel,
                                                     link2->coldetModel));
          }
      }
  }
  std::cout << "[" << m_profile.instance_name << "] " << m_checkPairs.size()
            << " pairs are defined" << std::endl;
  
  return RTC::RTC_OK;
}


/*
RTC::ReturnCode_t SelfCollisionChecker::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t SelfCollisionChecker::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t SelfCollisionChecker::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

RTC::ReturnCode_t SelfCollisionChecker::onActivated(RTC::UniqueId ec_id)
{
  std::cout << m_profile.instance_name<< ": onActivated(" << ec_id << ")" << std::endl;

  std::string fname = std::string(m_profile.instance_name) + ".log";
  m_log.open(fname.c_str());
  m_count = 0;

  return RTC::RTC_OK;
}

RTC::ReturnCode_t SelfCollisionChecker::onDeactivated(RTC::UniqueId ec_id)
{
  std::cout << m_profile.instance_name<< ": onDeactivated(" << ec_id << ")" << std::endl;

  return RTC::RTC_OK;
}

RTC::ReturnCode_t SelfCollisionChecker::onExecute(RTC::UniqueId ec_id)
{
  //std::cout << m_profile.instance_name<< ": onExecute(" << ec_id << ")" << std::endl;
    m_count++;
    if (m_count >= m_interval){
        m_count = 0;

        if (!m_qIn.isNew()) return RTC::RTC_OK;

        while (m_qIn.isNew()) m_qIn.read();
        for (unsigned int i=0; i<m_q.data.length(); i++){
            m_robot->joint(i)->q = m_q.data[i];
        }
        m_robot->calcForwardKinematics();
        for (int i=0; i<m_robot->numLinks(); i++){
            Link *l = m_robot->link(i);
            l->coldetModel->setPosition(l->R, l->p);
        }
        for (unsigned int i=0; i<m_checkPairs.size(); i++){
            if (m_checkPairs[i].checkCollision()){
                double tm = m_q.tm.sec + m_q.tm.nsec/1e9;
                m_log << tm << " " << m_checkPairs[i].model(0)->name()
                      << " " << m_checkPairs[i].model(1)->name() << std::endl;
            }
        }
    }

    return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t SelfCollisionChecker::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t SelfCollisionChecker::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t SelfCollisionChecker::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t SelfCollisionChecker::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t SelfCollisionChecker::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/



extern "C"
{

  void SelfCollisionCheckerInit(RTC::Manager* manager)
  {
    RTC::Properties profile(spec);
    manager->registerFactory(profile,
                             RTC::Create<SelfCollisionChecker>,
                             RTC::Delete<SelfCollisionChecker>);
  }

};


