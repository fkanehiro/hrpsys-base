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
    "conf.default.debugLevel", "0",
    ""
  };
// </rtc-template>

VirtualForceSensor::VirtualForceSensor(RTC::Manager* manager)
  : RTC::DataFlowComponentBase(manager),
    // <rtc-template block="initializer">
    m_qRefIn("qRef", m_qRef),
    m_qCurrentIn("qCurrent", m_qCurrent),
    m_tauIn("tau", m_tau),
    //m_VirtualForceSensorServicePort("VirtualForceSensorService"),
    // </rtc-template>
    m_debugLevel(0),
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
  bindParameter("debugLevel", m_debugLevel, "0");
  
  // </rtc-template>

  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("qRef", m_qRefIn);
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

  // virtual_force_sensor: <name>, <base>, <target>, 0, 0, 0,  0, 0, 0, 1
  coil::vstring virtual_force_sensor = coil::split(prop["virtual_force_sensor"], ",");
  for(int i = 0; i < virtual_force_sensor.size()/10; i++ ){
    std::string name = virtual_force_sensor[i*10+0];
    VirtualForceSensorParam p;
    p.base_name = virtual_force_sensor[i*10+1];
    p.target_name = virtual_force_sensor[i*10+2];
    hrp::dvector tr(7);
    for (int j = 0; j < 7; j++ ) {
      coil::stringTo(tr[j], virtual_force_sensor[i*10+3+j].c_str());
    }
    p.p = hrp::Vector3(tr[0], tr[1], tr[2]);
    p.R = (Eigen::Quaternion<double>(tr[3], tr[4], tr[5], tr[6])).toRotationMatrix();
    std::cerr << "virtual force sensor : " << name << std::endl;
    std::cerr << "                base : " << p.base_name << std::endl;
    std::cerr << "              target : " << p.target_name << std::endl;
    std::cerr << "                T, R : " << p.p[0] << " " << p.p[1] << " " << p.p[2] << std::endl << p.R << std::endl;
    p.path = hrp::JointPathPtr(new hrp::JointPath(m_robot->link(p.base_name), m_robot->link(p.target_name)));
    m_sensors[name] = p;
    if ( m_sensors[name].path->numJoints() == 0 ) {
      std::cerr << "ERROR : Unknown link path " << m_sensors[name].base_name << " " << m_sensors[name].target_name  << std::endl;
      return RTC::RTC_ERROR;
    }
  }
  int nforce = m_sensors.size();
  m_force.resize(nforce);
  m_forceOut.resize(nforce);
  int i = 0;
  std::map<std::string, VirtualForceSensorParam>::iterator it = m_sensors.begin();
  while ( it != m_sensors.end() ) {
    m_forceOut[i] = new OutPort<TimedDoubleSeq>((*it).first.c_str(), m_force[i]);
    m_force[i].data.length(6);
    registerOutPort((*it).first.c_str(), *m_forceOut[i]);
    it++; i++;
  }

  // set additional parameters for force calc
  m_error_to_torque_gain.resize(m_robot->numJoints());
  m_error_dead_zone.resize(m_robot->numJoints());
  coil::vstring error_to_torque_gain = coil::split(prop["error_to_torque_gain"], ",");  // gain for error to calculate torque
  assert(m_error_to_torque_gain.size() == error_to_torque_gain.size());
  coil::vstring error_dead_zone = coil::split(prop["error_dead_zone"], ",");   // dead zone of qCurrent - qRef in state of neutral
  assert(m_error_dead_zone.size() == error_dead_zone.size());
  std::cerr << "set virtual force sensor params" << std::endl;
  for (int i = 0; i < m_robot->numJoints(); i++ ) {
    coil::stringTo(m_error_to_torque_gain[i], error_to_torque_gain[i].c_str());
    coil::stringTo(m_error_dead_zone[i], error_dead_zone[i].c_str());
    std::cerr << m_robot->joint(i)->name << " : ";
    std::cerr << " gain " << m_error_to_torque_gain[i];
    std::cerr << " dead_zone " << m_error_dead_zone[i] << std::endl;
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

#define DEBUGP ((m_debugLevel==1 && loop%200==0) || m_debugLevel > 1 )
RTC::ReturnCode_t VirtualForceSensor::onExecute(RTC::UniqueId ec_id)
{
  //std::cout << m_profile.instance_name<< ": onExecute(" << ec_id << ")" << std::endl;
  static int loop = 0;
  loop ++;

  coil::TimeValue coiltm(coil::gettimeofday());
  RTC::Time tm;
  tm.sec = coiltm.sec();
  tm.nsec = coiltm.usec()*1000;

  if (m_qRefIn.isNew()) {
    m_qRefIn.read();
  }

  if (m_qCurrentIn.isNew()) {
    m_qCurrentIn.read();
  }
  if (m_tauIn.isNew()) {
    m_tauIn.read();
  }
  if ( m_qRef.data.length() ==  m_robot->numJoints() &&
       m_qCurrent.data.length() ==  m_robot->numJoints() &&
       m_tau.data.length() ==  m_robot->numJoints() ) {
    // reference model
    for ( int i = 0; i < m_robot->numJoints(); i++ ){
      m_robot->joint(i)->q = m_qCurrent.data[i];
    }
    m_robot->calcForwardKinematics();
    m_robot->calcCM();
    m_robot->rootLink()->calcSubMassCM();

    std::map<std::string, VirtualForceSensorParam>::iterator it = m_sensors.begin();
    int i = 0;
    while ( it != m_sensors.end() ) {

      hrp::JointPathPtr path = (*it).second.path;
      int n = path->numJoints();
      hrp::dmatrix J(6, n);
      hrp::dmatrix Jtinv(6, n);
      path->calcJacobian(J);
      hrp::calcPseudoInverse(J.transpose(), Jtinv);
      hrp::dvector torque(n);
      hrp::dvector force(6);

      // subm*g x (submwc/subm - p) . R*a
#define joint_torque(i) (path->joint(i)->subm*g).cross(path->joint(i)->submwc/path->joint(i)->subm - path->joint(i)->p).dot(path->joint(i)->R*path->joint(i)->a)
#define joint_error(i) (m_qCurrent.data[path->joint(i)->jointId]-m_qRef.data[path->joint(i)->jointId])
#define error2torque(i, offset) ((joint_error(i) > 0) ? (fmax(joint_error(i)-offset,0) * m_error_to_torque_gain[path->joint(i)->jointId]) : (fmin(joint_error(i)+offset,0) * m_error_to_torque_gain[path->joint(i)->jointId]))

      hrp::Vector3 g(0, 0, 9.8);
      if ( DEBUGP ) {
        std::cerr << "  sensor name  : " << it->first << std::endl;
        std::cerr << "sensor torque  : ";
        for (int j = 0; j < n; j++) {
          std::cerr << " " << m_tau.data[path->joint(j)->jointId] ;
        }
        std::cerr << std::endl;
        std::cerr << "  calc torque  : ";
        for (int j = 0; j < n; j++) {
          std::cerr << " " << joint_torque(j);
        }
        std::cerr << std::endl;
      }
      
      for (int j = 0; j < n; j++) {
        // torque calculation from electric current
        //torque[j] = m_tau.data[path->joint(j)->jointId] -
        
        // torque calclation from error of joint angle
        if ( m_error_to_torque_gain[path->joint(j)->jointId] == 0.0
             || fabs(joint_error(j)) < m_error_dead_zone[path->joint(j)->jointId] ) {
          torque[j] = 0;
        } else {
          torque[j] = error2torque(j, fabs(m_error_dead_zone[path->joint(j)->jointId])) - joint_torque(j);
        }
        
      }
      if ( DEBUGP ) {
        std::cerr << "row gear torque: ";
        for (int j = 0; j < n; j++) {
          std::cerr << " " << joint_error(j);
        }
        std::cerr << std::endl;

        std::cerr << "  gear torque  : ";
        for (int j = 0; j < n; j++) {
          std::cerr << " " << torque[j];
        }
        std::cerr << std::endl;
      }

      force = Jtinv * torque;
      // force = J * torque;

      if ( DEBUGP ) {
        std::cerr << "    raw force  : ";
        for ( int j = 0; j < 6; j ++ ) {
          std::cerr << " " << force[j] ;
        }
        std::cerr << std::endl;
      }

      hrp::dvector force_p(3), force_r(3);
      for ( int j = 0; j < 3; j ++ ) {
        force_p[j] = force[j];
        force_r[j] = force[j+3];
      }
      force_p = (*it).second.R * path->endLink()->R.transpose() * force_p;
      force_r = (*it).second.R * path->endLink()->R.transpose() * force_r;
      for ( int j = 0; j < 3; j ++ ) {
        m_force[i].data[j+0] = force_p[j];
        m_force[i].data[j+3] = force_r[j];
      }

      if ( DEBUGP ) {
        std::cerr << "        torque : ";
        for (int j = 0; j < n; j++) {
          std::cerr << " " << torque[j];
        }
        std::cerr << std::endl;
        std::cerr << " result force  : ";
        for ( int j = 0; j < 6; j ++ ) {
          std::cerr << " " << m_force[i].data[j] ;
        }
        std::cerr << std::endl;
      }

      m_force[i].tm = tm; // put timestamp
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


