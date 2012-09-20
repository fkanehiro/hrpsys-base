// -*- C++ -*-
/*!
 * @file  VirtualForceSensor.cpp
 * @brief virtual force sensor component
 * $Date$
 *
 * $Id$
 */

#include "VirtualForceSensor.h"
#include <rtm/CorbaNaming.h>
#include <hrpModel/ModelLoaderUtil.h>
#include <hrpUtil/MatrixSolvers.h>

// Module specification
// <rtc-template block="module_spec">
static const char* virtualforcesensor_spec[] =
  {
    "implementation_id", "VirtualForceSensor",
    "type_name",         "VirtualForceSensor",
    "description",       "null component",
    "version",           "1.0",
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

VirtualForceSensor::VirtualForceSensor(RTC::Manager* manager)
  : RTC::DataFlowComponentBase(manager),
    // <rtc-template block="initializer">
    m_qCurrentIn("qCurrent", m_qCurrent),
    m_tauIn("tau", m_tau),
    //m_VirtualForceSensorServicePort("VirtualForceSensorService"),
    // </rtc-template>
    dummy(0)
{
}

VirtualForceSensor::~VirtualForceSensor()
{
}



RTC::ReturnCode_t VirtualForceSensor::onInitialize()
{
  std::cout << m_profile.instance_name << ": onInitialize()" << std::endl;
  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  
  // </rtc-template>

  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("qCurrent", m_qCurrentIn);
  addInPort("tau", m_tauIn);

  // Set OutPort buffer
  
  // Set service provider to Ports
  //m_VirtualForceSensorServicePort.registerProvider("service0", "VirtualForceSensorService", m_VirtualForceSensorService);
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  //addPort(m_VirtualForceSensorServicePort);
  
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

  // <name>, <base>, <target>, 0, 0, 0,  0, 0, 0, 1
  coil::vstring virtual_force_sensor = coil::split(prop["virtual_force_sensor"], ",");
  for(int i = 0; i < virtual_force_sensor.size()/10; i++ ){
    std::string name = virtual_force_sensor[i*10+0];
    std::string base = virtual_force_sensor[i*10+1];
    std::string target = virtual_force_sensor[i*10+2];
    hrp::dvector tr(7);
    for (int j = 0; j < 7; j++ ) {
      coil::stringTo(tr[j], virtual_force_sensor[i*10+3+j].c_str());
    }
    std::cerr << "virtual force sensor : " << name << std::endl;
    std::cerr << "                base : " << base << std::endl;
    std::cerr << "              target : " << target << std::endl;
    std::cerr << "                T, R : " << tr[0] << " " << tr[1] << " "<< tr[2] << ", " << tr[3] << " " << tr[4] << " " << tr[5] << " " << tr[6] << " " << std::endl;
    m_sensors[name] = hrp::JointPathPtr(new hrp::JointPath(m_robot->link(base), m_robot->link(target)));
    if ( m_sensors[name]->numJoints() == 0 ) {
      std::cerr << "ERROR : Unknown link path " << base << " " << target << std::endl;
      return RTC::RTC_ERROR;
    }
  }
  int nforce = m_sensors.size();
  m_force.resize(nforce);
  m_forceOut.resize(nforce);
  int i = 0;
  std::map<std::string, hrp::JointPathPtr>::iterator it = m_sensors.begin();
  while ( it != m_sensors.end() ) {
    m_forceOut[i] = new OutPort<TimedDoubleSeq>((*it).first.c_str(), m_force[i]);
    m_force[i].data.length(6);
    registerOutPort((*it).first.c_str(), *m_forceOut[i]);
    it++; i++;
  }

  return RTC::RTC_OK;
}



/*
RTC::ReturnCode_t VirtualForceSensor::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t VirtualForceSensor::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t VirtualForceSensor::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

RTC::ReturnCode_t VirtualForceSensor::onActivated(RTC::UniqueId ec_id)
{
  std::cout << m_profile.instance_name<< ": onActivated(" << ec_id << ")" << std::endl;
  return RTC::RTC_OK;
}

RTC::ReturnCode_t VirtualForceSensor::onDeactivated(RTC::UniqueId ec_id)
{
  std::cout << m_profile.instance_name<< ": onDeactivated(" << ec_id << ")" << std::endl;
  return RTC::RTC_OK;
}

RTC::ReturnCode_t VirtualForceSensor::onExecute(RTC::UniqueId ec_id)
{
  //std::cout << m_profile.instance_name<< ": onExecute(" << ec_id << ")" << std::endl;

  if (m_qCurrentIn.isNew()) {
    m_qCurrentIn.read();
  }
  if (m_tauIn.isNew()) {
    m_tauIn.read();
  }
  if ( m_qCurrent.data.length() ==  m_robot->numJoints() &&
       m_tau.data.length() ==  m_robot->numJoints() ) {
    // reference model
    for ( int i = 0; i < m_robot->numJoints(); i++ ){
      m_robot->joint(i)->q = m_qCurrent.data[i];
    }
    m_robot->calcForwardKinematics();
    m_robot->calcCM();
    m_robot->rootLink()->calcSubMassCM();

    std::map<std::string, hrp::JointPathPtr>::iterator it = m_sensors.begin();
    int i = 0;
    while ( it != m_sensors.end() ) {

      hrp::JointPathPtr path = (*it).second;
      int n = path->numJoints();
      hrp::dmatrix J(6, n);
      hrp::dmatrix Jinv(6, n);
      path->calcJacobian(J);
      hrp::calcPseudoInverse(J.transpose(), Jinv);
      hrp::dvector torque(n);
      hrp::dvector force(6);

      hrp::Vector3 g(0, 0, 9.8);
#if 0
      std::cerr << "sensor torque  : ";
      for (int j = 0; j < n; j++) {
        std::cerr << " " << m_tau.data[path->joint(j)->jointId] ;
      }
      std::cerr << std::endl;
      // subm*g x (submwc/subm - p) . R*a
      std::cerr << "  calc torque  : ";
      for (int j = 0; j < n; j++) {
          std::cerr << " " << (path->joint(j)->subm*g).cross(path->joint(j)->submwc/path->joint(j)->subm - path->joint(j)->p).dot(path->joint(j)->R*path->joint(j)->a);
      }
      std::cerr << std::endl;
#endif

      for (int j = 0; j < n; j++) {
        torque[j] = m_tau.data[path->joint(j)->jointId] -
            (path->joint(j)->subm*g).cross(path->joint(j)->submwc/path->joint(j)->subm - path->joint(j)->p).dot(path->joint(j)->R*path->joint(j)->a);
      }
      force = Jinv * torque;
      for ( int j = 0; j < 6; j ++ ) {
        m_force[i].data[j] = force[j];
      }

#if 0
      std::cerr << "        torque : ";
      for (int j = 0; j < n; j++) {
        std::cerr << " " << torque[j];
      }
      std::cerr << std::endl;
      std::cerr << " result force  : ";
      for ( int j = 0; j < 6; j ++ ) {
        std::cerr << " " << force[j] ;
      }
      std::cerr << std::endl;
#endif

      m_forceOut[i]->write();

      it++; i++;
    }
    //
#if 0
  (:calc-force-from-joint-torque
   (limb all-torque &key (move-target (send self limb :end-coords)) (use-torso))
   (let* ((link-list
	   (send self :link-list
		 (send move-target :parent)
		 (unless use-torso (car (send self limb :links)))))
	  (jacobian
	   (send self :calc-jacobian-from-link-list
		 link-list
		 :move-target move-target
		 :rotation-axis (list t)
		 :translation-axis (list t)))
	  (torque (instantiate float-vector (length link-list))))
     (dotimes (i (length link-list))
       (setf (elt torque i)
	     (elt all-torque (position (send (elt link-list i) :joint) (send self :joint-list)))))
     (transform (send self :calc-inverse-jacobian (transpose jacobian))
		torque)))
#endif

  }
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t VirtualForceSensor::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t VirtualForceSensor::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t VirtualForceSensor::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t VirtualForceSensor::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t VirtualForceSensor::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/



extern "C"
{

  void VirtualForceSensorInit(RTC::Manager* manager)
  {
    RTC::Properties profile(virtualforcesensor_spec);
    manager->registerFactory(profile,
                             RTC::Create<VirtualForceSensor>,
                             RTC::Delete<VirtualForceSensor>);
  }

};


