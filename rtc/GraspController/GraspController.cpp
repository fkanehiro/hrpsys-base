// -*- C++ -*-
/*!
 * @file  GraspController.cpp
 * @brief soft error limiter
 * $Date$
 *
 * $Id$
 */

#include "GraspController.h"
#include "hrpsys/util/VectorConvert.h"
#include <rtm/CorbaNaming.h>
#include <hrpModel/ModelLoaderUtil.h>
#include "hrpsys/idl/RobotHardwareService.hh"

#include <hrpModel/Link.h>

#include <math.h>
#define deg2rad(x)((x)*M_PI/180)
#define min(a,b) ((a)<(b)?(a):(b))
#define max(a,b) ((a)>(b)?(a):(b))

// Module specification
// <rtc-template block="module_spec">
static const char* softerrorlimiter_spec[] =
  {
    "implementation_id", "GraspController",
    "type_name",         "GraspController",
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

GraspController::GraspController(RTC::Manager* manager)
  : RTC::DataFlowComponentBase(manager),
    // <rtc-template block="initializer">
    m_qRefIn("qRef", m_qRef),
    m_qCurrentIn("qCurrent", m_qCurrent),
    m_qIn("qIn", m_q),
    m_qOut("q", m_q),
    m_GraspControllerServicePort("GraspControllerService"),
    // </rtc-template>
    m_debugLevel(0),
    dummy(0)
{
  m_service0.grasp(this);
}

GraspController::~GraspController()
{
}



RTC::ReturnCode_t GraspController::onInitialize()
{
  std::cout << m_profile.instance_name << ": onInitialize()" << std::endl;
  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  bindParameter("debugLevel", m_debugLevel, "0");
  
  // </rtc-template>

  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("qRef", m_qRefIn); // for naming rule of hrpsys_config.py
  addInPort("qCurrent", m_qCurrentIn);
  addInPort("qIn", m_qIn);
  
  // Set OutPort buffer
  addOutPort("q", m_qOut); // for naming rule of hrpsys_config.py
  
  // Set service provider to Ports
  m_GraspControllerServicePort.registerProvider("service0", "GraspControllerService", m_service0);
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  addPort(m_GraspControllerServicePort);
  
  // </rtc-template>

  m_robot = hrp::BodyPtr(new hrp::Body());
  RTC::Properties& prop = getProperties();
  coil::stringTo(m_dt, prop["dt"].c_str());

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
      std::cerr << "[" << m_profile.instance_name << "] failed to load model[" << prop["model"] << "]" 
                << std::endl;
      return RTC::RTC_ERROR;
  }
  // <name> : <joint1>, <direction1>, <joint2>, <direction2>, <joint1>, <direction2>, <name> : <joint1>, <direction1>
  // check num of grasp
  coil::vstring grasp_joint_params = coil::split(prop["grasp_joint_groups"], ",");
  std::string grasp_name;
  GraspJoint grasp_joint;
  std::vector<GraspJoint> grasp_joints;
  for(unsigned int i = 0, f = 0; i < grasp_joint_params.size(); i++ ){
    coil::vstring grasp_joint_group_names = coil::split(grasp_joint_params[i], ":");
    if ( grasp_joint_group_names.size() > 1 ) {
      if ( grasp_name != "" ) {
        GraspParam grasp_param;
        grasp_param.time = 0;
        grasp_param.joints = grasp_joints;
        grasp_param.time = 1; // stop
        m_grasp_param[grasp_name] = grasp_param;
        grasp_joints.clear();
      }
      // initialize
      grasp_name = grasp_joint_group_names[0];
      if ( !! m_robot->link(grasp_joint_group_names[1]) ) {
        grasp_joint.id = m_robot->link(std::string(grasp_joint_group_names[1].c_str()))->jointId;
      }
      f = 0;
      i++;
    }
    if ( f == 0 ) {
      coil::stringTo(grasp_joint.dir,grasp_joint_params[i].c_str());
      grasp_joints.push_back(grasp_joint);
      f = 1 ;
    } else {
      if ( !! m_robot->link(grasp_joint_params[i]) ) {
        grasp_joint.id = m_robot->link(grasp_joint_params[i])->jointId;
      }
      f = 0 ;
    }
  }
  // finalize
  if ( grasp_name != "" ) {
    GraspParam grasp_param;
    grasp_param.time = 0;
    grasp_param.joints = grasp_joints;
    grasp_param.time = 1; // stop
    m_grasp_param[grasp_name] = grasp_param;
  }
  //
  if ( m_debugLevel ) {
    std::map<std::string, GraspParam >::iterator it = m_grasp_param.begin();
    while ( it != m_grasp_param.end() ) {
      std::cerr << "[" << m_profile.instance_name << "] " << it->first << " : ";
      for ( unsigned int i = 0 ; i < it->second.joints.size(); i++ ) {
        std::cerr << "id = " << it->second.joints[i].id << ", dir = " << it->second.joints[i].dir << ", ";
      }
      std::cerr << std::endl;
      it++;
    }
  }


  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t GraspController::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t GraspController::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t GraspController::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

RTC::ReturnCode_t GraspController::onActivated(RTC::UniqueId ec_id)
{
  std::cout << m_profile.instance_name<< ": onActivated(" << ec_id << ")" << std::endl;
  return RTC::RTC_OK;
}

RTC::ReturnCode_t GraspController::onDeactivated(RTC::UniqueId ec_id)
{
  std::cout << m_profile.instance_name<< ": onDeactivated(" << ec_id << ")" << std::endl;
  for (std::map<std::string, GraspParam >::iterator it = m_grasp_param.begin(); it != m_grasp_param.end(); it++ ) {
    it->second.time = 2; // count down to 1
    it->second.target_error = 0;
  }
  return RTC::RTC_OK;
}

RTC::ReturnCode_t GraspController::onExecute(RTC::UniqueId ec_id)
{
  //std::cout << m_profile.instance_name<< ": onExecute(" << ec_id << ")" << std::endl;
  if (m_qRefIn.isNew()) {
    m_qRefIn.read();
  }
  if (m_qCurrentIn.isNew()) {
    m_qCurrentIn.read();
  }
  if (m_qIn.isNew()) {
    m_qIn.read();
  }

  if ( m_qRef.data.length() == m_qCurrent.data.length() &&
       m_qRef.data.length() == m_q.data.length() ) {

    std::map<std::string, GraspParam >::iterator it = m_grasp_param.begin();
    while ( it != m_grasp_param.end() ) {
      GraspParam& grasp_param = it->second;
      if ( grasp_param.time < 0 ) { // staring
        grasp_param.time++;
      } else if ( grasp_param.time == 0 ) {// working
        //std::cerr << "grasp mode " << std::endl;
        for ( unsigned int j= 0; j < grasp_param.joints.size(); j++ ) {
          int i = grasp_param.joints[j].id;
          if ( 0 <= i && (unsigned int)i < m_qRef.data.length() ) {
            double error = (m_qCurrent.data[i] - m_qRef.data[i]) + grasp_param.target_error * grasp_param.joints[j].dir;
            double diff  = fabs(error);
            if ( error > 0 ) m_q.data[i] = m_qRef.data[i] + diff;
            if ( error < 0 ) m_q.data[i] = m_qRef.data[i] - diff;
            //std::cerr << "id = " << i << ", ref = " << m_qRef.data[i] << ", cur =" << m_qCurrent.data[i] << " error = " << error << ", diff = " << diff << ", q = " << m_q.data[i] << " (target = " << grasp_param.target_error << ", dir=" << grasp_param.joints[j].dir << ")" << std::endl;
          } else {
            if (m_debugLevel==1) std::cerr << "GraspController is not working..., id = " << i << std::endl;
          }
        }
      } else if ( grasp_param.time > 1 ) { // stopping 
        grasp_param.time--;
        for ( unsigned int j= 0; j < grasp_param.joints.size(); j++ ) {
          int i = grasp_param.joints[j].id;
          if ( 0 <= i && (unsigned int)i < m_qRef.data.length() ) {
            m_qRef.data[i] = (m_qRef.data[i] - m_q.data[i] ) * grasp_param.time/1000 + m_q.data[i];
            double diff = m_qRef.data[i] - m_qCurrent.data[i];
            if ( diff > 0 ) diff = min(diff, 0.034907); // 2 [deg]
            if ( diff < 0 ) diff = max(diff,-0.034907); // 2 [deg]
            m_q.data[i] = diff + m_qCurrent.data[i];
            //std::cerr << "id = " << i << ", ref = " << m_qRef.data[i] << ", cur =" << m_qCurrent.data[i] << " q = " << m_q.data[i] << std::endl;
          }
        }
      } else if ( grasp_param.time == 1 ) {// stop
      }
      it++;
    }

    m_qOut.write();
  }else if ( m_qCurrent.data.length() == m_q.data.length() ) {
    if (m_debugLevel==1) std::cerr << "GraspController in pass through mode..." << std::endl;
    m_qOut.write();
  } else {
    std::cerr << "GraspController is not working..." << std::endl;
    std::cerr << "          m_qIn " << m_q.data.length() << std::endl;
    std::cerr << "         m_qRef " << m_qRef.data.length() << std::endl;
    std::cerr << "     m_qCurrent " << m_qCurrent.data.length() << std::endl;
  }

  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t GraspController::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t GraspController::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t GraspController::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t GraspController::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t GraspController::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

 // m_grasp_param.time
 // > 1  : stopping
 //   1  : stopped
 //   0  : working
 // < 0  : starting
bool GraspController::startGrasp(const char *name, double target_error) {
  if ( m_grasp_param.find( name ) == m_grasp_param.end() ) {
    std::cerr << "[" << m_profile.instance_name << "] Could not found grasp controller " << name << std::endl;
    return false;
  }
  std::cerr << "[" << m_profile.instance_name << "] Start Grasp " << name << std::endl;
  m_grasp_param[name].time = -10; // count up to 0
  m_grasp_param[name].target_error = fabs(target_error);
  return true;
}

bool GraspController::stopGrasp(const char *name) {
  if ( m_grasp_param.find( name ) == m_grasp_param.end() ) {
    std::cerr << "[" << m_profile.instance_name << "] Could not found grasp controller " << name << std::endl;
    return false;
  }
  std::cerr << "[" << m_profile.instance_name << "] Stop Grasp " << name << std::endl;
  m_grasp_param[name].time = 1000; // count down to 1
  m_grasp_param[name].target_error = 0;
  return true;
}


extern "C"
{

  void GraspControllerInit(RTC::Manager* manager)
  {
    RTC::Properties profile(softerrorlimiter_spec);
    manager->registerFactory(profile,
                             RTC::Create<GraspController>,
                             RTC::Delete<GraspController>);
  }

};


