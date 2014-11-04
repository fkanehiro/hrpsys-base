// -*- C++ -*-
/*!
 * @file  ImpedanceController.cpp
 * @brief impedance controller component
 * $Date$
 *
 * $Id$
 */

#include <rtm/CorbaNaming.h>
#include <hrpModel/Link.h>
#include <hrpModel/Sensor.h>
#include <hrpModel/ModelLoaderUtil.h>
#include "ImpedanceController.h"
#include "JointPathEx.h"
#include <hrpModel/JointPath.h>
#include <hrpUtil/MatrixSolvers.h>
#include "util/Hrpsys.h"


#define MAX_TRANSITION_COUNT (2/m_dt)
typedef coil::Guard<coil::Mutex> Guard;

// Module specification
// <rtc-template block="module_spec">
static const char* impedancecontroller_spec[] =
    {
        "implementation_id", "ImpedanceController",
        "type_name",         "ImpedanceController",
        "description",       "impedance controller component",
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

ImpedanceController::ImpedanceController(RTC::Manager* manager)
    : RTC::DataFlowComponentBase(manager),
      // <rtc-template block="initializer">
      m_qCurrentIn("qCurrent", m_qCurrent),
      m_qRefIn("qRef", m_qRef),
      m_rpyIn("rpy", m_rpy),
      m_rpyRefIn("rpyRef", m_rpyRef),
      m_qOut("q", m_q),
      m_ImpedanceControllerServicePort("ImpedanceControllerService"),
      // </rtc-template>
      m_robot(hrp::BodyPtr()),
      m_debugLevel(0),
      dummy(0)
{
    m_service0.impedance(this);
}

ImpedanceController::~ImpedanceController()
{
}


RTC::ReturnCode_t ImpedanceController::onInitialize()
{
    std::cout << "ImpedanceController::onInitialize()" << std::endl;
    bindParameter("debugLevel", m_debugLevel, "0");

    // Registration: InPort/OutPort/Service
    // <rtc-template block="registration">
    // Set InPort buffers
    addInPort("qCurrent", m_qCurrentIn);
    addInPort("qRef", m_qRefIn);
    addInPort("rpy", m_rpyIn);
    addInPort("rpyRef", m_rpyRefIn);

    // Set OutPort buffer
    addOutPort("q", m_qOut);
  
    // Set service provider to Ports
    m_ImpedanceControllerServicePort.registerProvider("service0", "ImpedanceControllerService", m_service0);
  
    // Set service consumers to Ports
  
    // Set CORBA Service Ports
    addPort(m_ImpedanceControllerServicePort);
  
    // </rtc-template>
    // <rtc-template block="bind_config">
    // Bind variables and configuration variable
  
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

    coil::vstring virtual_force_sensor = coil::split(prop["virtual_force_sensor"], ",");
    int npforce = m_robot->numSensors(hrp::Sensor::FORCE);
    int nvforce = virtual_force_sensor.size()/10;
    int nforce  = npforce + nvforce;
    m_force.resize(nforce);
    m_forceIn.resize(nforce);
    m_ref_force.resize(nforce);
    m_ref_forceIn.resize(nforce);
    for (unsigned int i=0; i<npforce; i++){
        hrp::Sensor *s = m_robot->sensor(hrp::Sensor::FORCE, i);
        // actual inport
        m_forceIn[i] = new InPort<TimedDoubleSeq>(s->name.c_str(), m_force[i]);
        m_force[i].data.length(6);
        registerInPort(s->name.c_str(), *m_forceIn[i]);
        // ref inport
        m_ref_force[i].data.length(6);
        m_ref_forceIn[i] = new InPort<TimedDoubleSeq>(std::string("ref_"+s->name+"In").c_str(), m_ref_force[i]);
        registerInPort(std::string("ref_"+s->name+"In").c_str(), *m_ref_forceIn[i]);
        std::cerr << "[" << m_profile.instance_name << "] force sensor" << std::endl;
        std::cerr << "[" << m_profile.instance_name << "]   name = " << s->name << std::endl;
    }
    for (unsigned int i=0; i<nvforce; i++){
        std::string name = virtual_force_sensor[i*10+0];
        VirtualForceSensorParam p;
        hrp::dvector tr(7);
        for (int j = 0; j < 7; j++ ) {
          coil::stringTo(tr[j], virtual_force_sensor[i*10+3+j].c_str());
        }
        p.p = hrp::Vector3(tr[0], tr[1], tr[2]);
        p.R = Eigen::AngleAxis<double>(tr[6], hrp::Vector3(tr[3],tr[4],tr[5])).toRotationMatrix(); // rotation in VRML is represented by axis + angle
        p.parent_link_name = virtual_force_sensor[i*10+2];
        m_sensors[name] = p;
        std::cerr << "[" << m_profile.instance_name << "] virtual force sensor" << std::endl;
        std::cerr << "[" << m_profile.instance_name << "]   name = " << name << ", parent = " << p.parent_link_name << std::endl;
        std::cerr << "[" << m_profile.instance_name << "]   localP = " << p.p.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[m]" << std::endl;
        std::cerr << "[" << m_profile.instance_name << "]   localR = " << p.R.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", "\n", "    [", "]")) << std::endl;

        m_forceIn[i+npforce] = new InPort<TimedDoubleSeq>(name.c_str(), m_force[i+npforce]);
        m_force[i+npforce].data.length(6);
        registerInPort(name.c_str(), *m_forceIn[i+npforce]);
        m_ref_force[i+npforce].data.length(6);
        for (unsigned int j=0; j<6; j++) m_ref_force[i].data[j] = 0.0;
        std::cerr << name << std::endl;
    }
    for (unsigned int i=0; i<m_forceIn.size(); i++){
      abs_forces.insert(std::pair<std::string, hrp::Vector3>(m_forceIn[i]->name(), hrp::Vector3::Zero()));
      abs_moments.insert(std::pair<std::string, hrp::Vector3>(m_forceIn[i]->name(), hrp::Vector3::Zero()));
    }

    unsigned int dof = m_robot->numJoints();
    for ( int i = 0 ; i < dof; i++ ){
      if ( i != m_robot->joint(i)->jointId ) {
        std::cerr << "[" << m_profile.instance_name << "] jointId is not equal to the index number" << std::endl;
        return RTC::RTC_ERROR;
      }
    }

    // allocate memory for outPorts
    m_q.data.length(dof);
    qrefv.resize(dof);
    loop = 0;

    return RTC::RTC_OK;
}



RTC::ReturnCode_t ImpedanceController::onFinalize()
{
    return RTC::RTC_OK;
}

/*
  RTC::ReturnCode_t ImpedanceController::onStartup(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t ImpedanceController::onShutdown(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

RTC::ReturnCode_t ImpedanceController::onActivated(RTC::UniqueId ec_id)
{
    std::cout << "ImpedanceController::onActivated(" << ec_id << ")" << std::endl;
    
    return RTC::RTC_OK;
}

RTC::ReturnCode_t ImpedanceController::onDeactivated(RTC::UniqueId ec_id)
{
  std::cout << "ImpedanceController::onDeactivated(" << ec_id << ")" << std::endl;
  Guard guard(m_mutex);
  for ( std::map<std::string, ImpedanceParam>::iterator it = m_impedance_param.begin(); it != m_impedance_param.end(); it++ ) {
    deleteImpedanceController(it->first);
    m_impedance_param[it->first].transition_count = 1;
  }
  return RTC::RTC_OK;
}

#define DEBUGP ((m_debugLevel==1 && loop%200==0) || m_debugLevel > 1 )
RTC::ReturnCode_t ImpedanceController::onExecute(RTC::UniqueId ec_id)
{
    //std::cout << "ImpedanceController::onExecute(" << ec_id << ")" << std::endl;
    loop ++;

    // check dataport input
    bool update_rpy = false;
    for (unsigned int i=0; i<m_forceIn.size(); i++){
        if ( m_forceIn[i]->isNew() ) {
            m_forceIn[i]->read();
        }
        if ( m_ref_forceIn[i]->isNew() ) {
            m_ref_forceIn[i]->read();
        }
    }
    if (m_rpyIn.isNew()) {
      m_rpyIn.read();
      update_rpy = true;
    }
    if (m_qCurrentIn.isNew()) {
      m_qCurrentIn.read();
      //
      if (update_rpy) updateRootLinkPosRot(m_rpy);
      for (unsigned int i=0; i<m_forceIn.size(); i++){
        if ( m_force[i].data.length()==6 ) {
          std::string sensor_name = m_forceIn[i]->name();
          hrp::ForceSensor* sensor = m_robot->sensor<hrp::ForceSensor>(sensor_name);
          hrp::Vector3 data_p(m_force[i].data[0], m_force[i].data[1], m_force[i].data[2]);
          hrp::Vector3 data_r(m_force[i].data[3], m_force[i].data[4], m_force[i].data[5]);
          hrp::Vector3 ref_data_p(m_ref_force[i].data[0], m_ref_force[i].data[1], m_ref_force[i].data[2]);
          hrp::Vector3 ref_data_r(m_ref_force[i].data[3], m_ref_force[i].data[4], m_ref_force[i].data[5]);
          if ( DEBUGP ) {
            std::cerr << "[" << m_profile.instance_name << "] force and moment [" << sensor_name << "]" << std::endl;
            std::cerr << "[" << m_profile.instance_name << "]   sensor force  = " << data_p.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[N]" << std::endl;
            std::cerr << "[" << m_profile.instance_name << "]   sensor moment = " << data_r.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[Nm]" << std::endl;
            std::cerr << "[" << m_profile.instance_name << "]   reference force  = " << ref_data_p.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[N]" << std::endl;
            std::cerr << "[" << m_profile.instance_name << "]   reference moment = " << ref_data_r.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[Nm]" << std::endl;
          }
          hrp::Matrix33 sensorR;
          if ( sensor ) {
            // real force sensore
            sensorR = sensor->link->R * sensor->localR;
          } else if ( m_sensors.find(sensor_name) !=  m_sensors.end()) {
            // virtual force sensor
            if ( DEBUGP ) {
              std::cerr << "[" << m_profile.instance_name << "]   sensorR = " << m_sensors[sensor_name].R.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", "\n", "    [", "]")) << std::endl;
            }
            sensorR = m_robot->link(m_sensors[sensor_name].parent_link_name)->R * m_sensors[sensor_name].R;
          } else {
            std::cerr << "[" << m_profile.instance_name << "]   unknown force param" << std::endl;
          }
          abs_forces[sensor_name] = sensorR * data_p;
          abs_moments[sensor_name] = sensorR * data_r;
          abs_ref_forces[sensor_name] = sensorR * ref_data_p;
          abs_ref_moments[sensor_name] = sensorR * ref_data_r;
          if ( DEBUGP ) {
            std::cerr << "[" << m_profile.instance_name << "]   abs force  = " << abs_forces[sensor_name].format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[N]" << std::endl;
            std::cerr << "[" << m_profile.instance_name << "]   abs moment = " << abs_moments[sensor_name].format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[Nm]" << std::endl;
            std::cerr << "[" << m_profile.instance_name << "]   abs ref force  = " << abs_ref_forces[sensor_name].format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[N]" << std::endl;
            std::cerr << "[" << m_profile.instance_name << "]   abs ref moment = " << abs_ref_moments[sensor_name].format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[Nm]" << std::endl;
          }
        }
      }
    }
    if (m_qRefIn.isNew()) {
        m_qRefIn.read();
        m_q.tm = m_qRef.tm;
    }
    if ( m_qRef.data.length() ==  m_robot->numJoints() &&
         m_qCurrent.data.length() ==  m_robot->numJoints() ) {

        if ( DEBUGP ) {
          std::cerr << "[" << m_profile.instance_name << "] qRef = ";
            for ( int i = 0; i <  m_qRef.data.length(); i++ ){
                std::cerr << " " << m_qRef.data[i];
            }
            std::cerr << std::endl;
        }

        if ( m_impedance_param.size() == 0 ) {
          for ( int i = 0; i < m_qRef.data.length(); i++ ){
            m_q.data[i] = m_qRef.data[i];
          }
          m_qOut.write();
          return RTC_OK;
        }

        Guard guard(m_mutex);

	{
	  hrp::dvector qorg(m_robot->numJoints());
	  
	  // reference model
	  for ( int i = 0; i < m_robot->numJoints(); i++ ){
	    qorg[i] = m_robot->joint(i)->q;
            m_robot->joint(i)->q = m_qRef.data[i];
            qrefv[i] = m_qRef.data[i];
	  }
          if (m_rpyRefIn.isNew()) {
            m_rpyRefIn.read();
            //updateRootLinkPosRot(m_rpyRef);
          }
	  m_robot->calcForwardKinematics();

	  // set sequencer position to target_p0
	  for ( std::map<std::string, ImpedanceParam>::iterator it = m_impedance_param.begin(); it != m_impedance_param.end(); it++ ) {
            ImpedanceParam& param = it->second;
            param.target_p0 = m_robot->link(param.target_name)->p;
            param.target_r0 = m_robot->link(param.target_name)->R;
          }
          // back to impedance robot model (only for controlled joint)
	  for ( std::map<std::string, ImpedanceParam>::iterator it = m_impedance_param.begin(); it != m_impedance_param.end(); it++ ) {
            ImpedanceParam& param = it->second;
            for ( int j = 0; j < param.manip->numJoints(); j++ ){
              int i = param.manip->joint(j)->jointId;
              m_robot->joint(i)->q = qorg[i];
            }
	  }
          if (update_rpy) updateRootLinkPosRot(m_rpy);
	  m_robot->calcForwardKinematics();

	}

	// set m_robot to qRef when deleting status
    std::map<std::string, ImpedanceParam>::iterator it = m_impedance_param.begin();
	while(it != m_impedance_param.end()){
	  std::string sensor_name = it->first;
	  ImpedanceParam& param = it->second;

	  if ( param.transition_count > 0 ) {
	    hrp::JointPathExPtr manip = param.manip;
	    for ( int j = 0; j < manip->numJoints(); j++ ) {
              int i = manip->joint(j)->jointId; // index in robot model
	      hrp::Link* joint =  m_robot->joint(i);
              // transition_smooth_gain moves from 0 to 1
              // (/ (log (/ (- 1 0.99) 0.99)) 0.5)
              double transition_smooth_gain = 1/(1+exp(-9.19*(((MAX_TRANSITION_COUNT - param.transition_count) / MAX_TRANSITION_COUNT) - 0.5)));
              joint->q = ( m_qRef.data[i] - param.transition_joint_q[i] ) * transition_smooth_gain + param.transition_joint_q[i];
	    }
        param.transition_count--;
	    if(param.transition_count <= 0){ // erase impedance param
          std::cerr << "[" << m_profile.instance_name << "] Finished cleanup and erase impedance param " << sensor_name << std::endl;
          m_impedance_param.erase(it++);
        }else{
          it++;
        }
	  } else {
	    // use impedance model

            hrp::Link* base = m_robot->link(param.base_name);
            hrp::Link* target = m_robot->link(param.target_name);
            assert(target);
            assert(base);

            param.current_p0 = target->p;
            param.current_r0 = target->R;

            hrp::JointPathExPtr manip = param.manip;
            assert(manip);
            //const int n = manip->numJoints();

            hrp::Vector3 dif_pos = hrp::Vector3(0,0,0);
            hrp::Vector3 vel_pos0 = hrp::Vector3(0,0,0);
            hrp::Vector3 vel_pos1 = hrp::Vector3(0,0,0);
            hrp::Vector3 dif_target_pos = hrp::Vector3(0,0,0);
            hrp::Vector3 dif_rot = hrp::Vector3(0,0,0);
            hrp::Vector3 vel_rot0 = hrp::Vector3(0,0,0);
            hrp::Vector3 vel_rot1 = hrp::Vector3(0,0,0);
            hrp::Vector3 dif_target_rot = hrp::Vector3(0,0,0);

            // rats/plugins/impedancecontrol.cpp
            //double M = 5, D = 100, K = 200;
            // dif_pos  = target_p0 (target_coords0) - current_p0(move_coords)
            // vel_pos0 = current_p0(move_coors) - current_p1(prev_coords0)
            // vel_pos1 = current_p1(prev_coords0) - current_p2(prev_coords1)
            // dif_target  = target_p0(target_coords0) - target_p1(target_coords1)
            //
            // current_p2(prev_coords1) = current_p1(prev_coords0)
            // currnet_p1(prev_coords0) = current_p0(move_coords) + vel_p
            // target_p1(target_coords1) = target_p0(target_coords0)

            if ( DEBUGP ) {
              std::cerr << "[" << m_profile.instance_name << "] impedance calc [" << it->first << "]" << std::endl;
              std::cerr << "[" << m_profile.instance_name << "]   cur0 = " << param.current_p0.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[m]" << std::endl;
              std::cerr << "[" << m_profile.instance_name << "]   cur1 = " << param.current_p1.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[m]" << std::endl;
              std::cerr << "[" << m_profile.instance_name << "]   cur2 = " << param.current_p2.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[m]" << std::endl;
              std::cerr << "[" << m_profile.instance_name << "]   tgt0 = " << param.target_p0.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[m]" << std::endl;
              std::cerr << "[" << m_profile.instance_name << "]   tgt1 = " << param.target_p1.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[m]" << std::endl;
            }

            dif_pos  = param.target_p0 - param.current_p0;
            vel_pos0 = param.current_p0 - param.current_p1;
            vel_pos1 = param.current_p1 - param.current_p2;
            dif_target_pos = param.target_p0 - param.target_p1;

            rats::difference_rotation(dif_rot, param.current_r0, param.target_r0);
            rats::difference_rotation(vel_rot0, param.current_r1, param.current_r0);
            rats::difference_rotation(vel_rot1, param.current_r2, param.current_r1);
            rats::difference_rotation(dif_target_rot, param.target_r1, param.target_r0);

            if ( DEBUGP ) {
                std::cerr << "[" << m_profile.instance_name << "]   dif_p  = " << dif_pos.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[m]" << std::endl;
                std::cerr << "[" << m_profile.instance_name << "]   vel_p0 = " << vel_pos0.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[m]" << std::endl;
                std::cerr << "[" << m_profile.instance_name << "]   vel_p1 = " << vel_pos1.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[m]" << std::endl;
                std::cerr << "[" << m_profile.instance_name << "]   dif_t  = " << dif_target_pos.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[m]" << std::endl;
                std::cerr << "[" << m_profile.instance_name << "]   dif_r  = " << dif_rot.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[rad]" << std::endl;
                std::cerr << "[" << m_profile.instance_name << "]   vel_r0 = " << vel_rot0.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[rad]" << std::endl;
                std::cerr << "[" << m_profile.instance_name << "]   vel_r1 = " << vel_rot1.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[rad]" << std::endl;
                std::cerr << "[" << m_profile.instance_name << "]   dif_t  = " << dif_target_rot.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[rad]" << std::endl;
            }
            hrp::Vector3 vel_p, vel_r;
            //std::cerr << "MDK = " << param.M_p << " " << param.D_p << " " << param.K_p << std::endl;
            //std::cerr << "MDK = " << param.M_r << " " << param.D_r << " " << param.K_r << std::endl;
            // std::cerr << "ref_force = " << param.ref_force[0] << " " << param.ref_force[1] << " " << param.ref_force[2] << std::endl;
            // std::cerr << "ref_moment = " << param.ref_moment[0] << " " << param.ref_moment[1] << " " << param.ref_moment[2] << std::endl;

            // ref_force/ref_moment and force_gain/moment_gain are expressed in global coordinates. 
            vel_p =  ( param.force_gain * (abs_forces[it->first] - abs_ref_forces[it->first]) * m_dt * m_dt
                       + param.M_p * ( vel_pos1 - vel_pos0 )
                       + param.D_p * ( dif_target_pos - vel_pos0 ) * m_dt
                       + param.K_p * ( dif_pos * m_dt * m_dt ) ) /
                     (param.M_p + (param.D_p * m_dt) + (param.K_p * m_dt * m_dt));
            vel_r =  ( param.moment_gain * (abs_moments[it->first] - abs_ref_moments[it->first]) * m_dt * m_dt
                       + param.M_r * ( vel_rot1 - vel_rot0 )
                       + param.D_r * ( dif_target_rot - vel_rot0 ) * m_dt
                       + param.K_r * ( dif_rot * m_dt * m_dt  ) ) /
                     (param.M_r + (param.D_r * m_dt) + (param.K_r * m_dt * m_dt));

            // generate smooth motion just after impedance started
            if ( DEBUGP ) {
                std::cerr << "[" << m_profile.instance_name << "]   vel_p  = " << vel_p.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[m]" << std::endl;
                std::cerr << "[" << m_profile.instance_name << "]   vel_r  = " << vel_r.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[rad]" << std::endl;
            }
            manip->calcInverseKinematics2Loop(vel_p, vel_r, 1.0, param.avoid_gain, param.reference_gain, &qrefv);

	    param.current_p2 = param.current_p1;
	    param.current_p1 = param.current_p0 + vel_p;
	    param.target_p1 = param.target_p0;

	    param.current_r2 = param.current_r1;
            // if ( std::fabs(vel_r.norm() - 0.0) < ::std::numeric_limits<double>::epsilon() ) {
            if ( vel_r.norm() != 0.0 ) {
              hrp::Matrix33 tmpm;
              Eigen::AngleAxis<double> tmpr(vel_r.norm(), vel_r.normalized());
              rats::rotm3times(tmpm, tmpr.toRotationMatrix(), param.current_r0);
              param.current_r1 = tmpm;
            } else {
              param.current_r1 = param.current_r0;
            }
            param.target_r1 = param.target_r0;

            if ( param.transition_count < 0 ) {
              param.transition_count++;
            }
        it++;
	  } // else
    } // while

        if ( m_q.data.length() != 0 ) { // initialized
            for ( int i = 0; i < m_robot->numJoints(); i++ ){
                m_q.data[i] = m_robot->joint(i)->q;
            }
            m_qOut.write();
            if ( DEBUGP ) {
                std::cerr << "[" << m_profile.instance_name << "] q = ";
                for ( int i = 0; i < m_q.data.length(); i++ ){
                    std::cerr << " " << m_q.data[i];
                }
                std::cerr << std::endl;
            }
        }
    } else {
        if ( DEBUGP || loop % 100 == 0 ) {
            std::cerr << "ImpedanceController is not working..." << std::endl;
            std::cerr << "         m_qRef " << m_qRef.data.length() << std::endl;
            std::cerr << "     m_qCurrent " << m_qCurrent.data.length() << std::endl;
        }
    }
    return RTC::RTC_OK;
}

/*
  RTC::ReturnCode_t ImpedanceController::onAborting(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t ImpedanceController::onError(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t ImpedanceController::onReset(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t ImpedanceController::onStateUpdate(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t ImpedanceController::onRateChanged(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

//
bool ImpedanceController::setImpedanceControllerParam(const std::string& i_name_, OpenHRP::ImpedanceControllerService::impedanceParam i_param_)
{
    std::string name = std::string(i_name_);
    std::string base_name = std::string(i_param_.base_name);
    std::string target_name = std::string(i_param_.target_name);
    if (base_name == "") base_name = m_robot->rootLink()->name;

    // wait to finish deleting if the target impedance param has been deleted
    if(m_impedance_param.find(name) != m_impedance_param.end() 
       && m_impedance_param[name].transition_count > 0){
      std::cerr << "[" << m_profile.instance_name << "] Wait to finish deleting old " << name << std::endl;
      waitDeletingImpedanceController(name);
    }

    // Lock Mutex
    Guard guard(m_mutex);
    
    if ( m_qRef.data.length() !=  m_robot->numJoints() ) {
      std::cerr << "[" << m_profile.instance_name << "] m_qRef has wrong size, m_robot->numJoints() = " << m_robot->numJoints() << ", m_qRef.data.length() = " << m_qRef.data.length() << std::endl;
        return false;
    }
    if ( m_qCurrent.data.length() !=  m_robot->numJoints() ) {
      std::cerr << "[" << m_profile.instance_name << "] m_qCurrent has wrong size, m_robot->numJoints() = " << m_robot->numJoints() << ", m_qCurrent.data.length() = " << m_qCurrent.data.length() << std::endl;
        return false;
    }

    int force_id = -1;
    if ( !checkImpedanceNameValidity (force_id, name) ) {
      return false;
    }

    if ( m_impedance_param.find(name) == m_impedance_param.end() ) {
        std::cerr << "[" << m_profile.instance_name << "] Set new impedance parameters" << std::endl;

	if ( ! m_robot->link(base_name) ) {
          std::cerr << "[" << m_profile.instance_name << "] Could not found link " << base_name << std::endl;
	  return false;
	}
	if ( ! m_robot->link(target_name) ) {
          std::cerr << "[" << m_profile.instance_name << "] Could not found link " << target_name << std::endl;
	  return false;
	}
    
	// set param
	ImpedanceParam p;
	p.base_name = base_name;
	p.target_name = target_name;
	p.M_p = i_param_.M_p;
	p.D_p = i_param_.D_p;
	p.K_p = i_param_.K_p;
	p.M_r = i_param_.M_r;
	p.D_r = i_param_.D_r;
	p.K_r = i_param_.K_r;

    
	// joint path
	p.manip = hrp::JointPathExPtr(new hrp::JointPathEx(m_robot, m_robot->link(p.base_name), m_robot->link(p.target_name)));

        if ( ! p.manip ) {
          std::cerr << "[" << m_profile.instance_name << "] invalid joint path from " << p.base_name << " to " << p.target_name << std::endl;
          return false;
        }

	// update reference model
        for (int i = 0; i < m_robot->numJoints(); i++ ) {
          // if other controller is already taken the joint, do not update the reference model
          bool update = true;
          for ( std::map<std::string, ImpedanceParam>::iterator it = m_impedance_param.begin(); it != m_impedance_param.end(); it++ ) {
            ImpedanceParam& param = it->second;
            for ( int j = 0; j < param.manip->numJoints(); j++ ){
              if ( i == param.manip->joint(j)->jointId ) update = false;
            }
          }
          if ( update ) m_robot->joint(i)->q = m_qCurrent.data[i];
        }
	m_robot->calcForwardKinematics();

        p.transition_joint_q.resize(m_robot->numJoints());

	p.target_p0 = m_robot->link(p.target_name)->p;
	p.target_p1 = m_robot->link(p.target_name)->p;
        p.target_r0 = m_robot->link(p.target_name)->R;
        p.target_r1 = m_robot->link(p.target_name)->R;

	p.current_p0 = m_robot->link(p.target_name)->p;
	p.current_p1 = m_robot->link(p.target_name)->p;
	p.current_p2 = m_robot->link(p.target_name)->p;
	p.current_r0 = m_robot->link(p.target_name)->R;
        p.current_r1 = m_robot->link(p.target_name)->R;
        p.current_r2 = m_robot->link(p.target_name)->R;
        p.transition_count = -MAX_TRANSITION_COUNT; // when start impedance, count up to 0

	m_impedance_param[name] = p;

    } else {
        std::cerr << "[" << m_profile.instance_name << "] Update impedance parameters" << std::endl;
    }

    m_impedance_param[name].sr_gain    = i_param_.sr_gain;
    m_impedance_param[name].avoid_gain = i_param_.avoid_gain;
    m_impedance_param[name].reference_gain = i_param_.reference_gain;
    m_impedance_param[name].manipulability_limit = i_param_.manipulability_limit;
    m_impedance_param[name].manip->setSRGain(m_impedance_param[name].sr_gain);
    m_impedance_param[name].manip->setManipulabilityLimit(m_impedance_param[name].manipulability_limit);

    m_impedance_param[name].M_p = i_param_.M_p;
    m_impedance_param[name].D_p = i_param_.D_p;
    m_impedance_param[name].K_p = i_param_.K_p;
    m_impedance_param[name].M_r = i_param_.M_r;
    m_impedance_param[name].D_r = i_param_.D_r;
    m_impedance_param[name].K_r = i_param_.K_r;

    m_impedance_param[name].force_gain = hrp::Vector3(i_param_.force_gain[0], i_param_.force_gain[1], i_param_.force_gain[2]).asDiagonal();
    m_impedance_param[name].moment_gain = hrp::Vector3(i_param_.moment_gain[0], i_param_.moment_gain[1], i_param_.moment_gain[2]).asDiagonal();

    for ( std::map<std::string, ImpedanceParam>::iterator it = m_impedance_param.begin(); it != m_impedance_param.end(); it++ ) {
      ImpedanceParam& param = it->second;
      std::cerr << "[" << m_profile.instance_name << "] set parameters" << std::endl;
      std::cerr << "[" << m_profile.instance_name << "]             name : " << it->first << std::endl;
      std::cerr << "[" << m_profile.instance_name << "]        base_name : " << param.base_name << std::endl;
      std::cerr << "[" << m_profile.instance_name << "]      target_name : " << param.target_name << std::endl;
      std::cerr << "[" << m_profile.instance_name << "]    M, D, K (pos) : " << param.M_p << " " << param.D_p << " " << param.K_p << std::endl;
      std::cerr << "[" << m_profile.instance_name << "]    M, D, K (rot) : " << param.M_r << " " << param.D_r << " " << param.K_r << std::endl;
      std::cerr << "[" << m_profile.instance_name << "]       force_gain : " << param.force_gain.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", "\n", "    [", "]")) << std::endl;
      std::cerr << "[" << m_profile.instance_name << "]      moment_gain : " << param.moment_gain.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", "\n", "    [", "]")) << std::endl;
      std::cerr << "[" << m_profile.instance_name << "]      manip_limit : " << param.manipulability_limit << std::endl;
      std::cerr << "[" << m_profile.instance_name << "]          sr_gain : " << param.sr_gain << std::endl;
      std::cerr << "[" << m_profile.instance_name << "]       avoid_gain : " << param.avoid_gain << std::endl;
      std::cerr << "[" << m_profile.instance_name << "]   reference_gain : " << param.reference_gain << std::endl;
    }

    
    return true;
}

void ImpedanceController::copyImpedanceParam (ImpedanceControllerService::impedanceParam& i_param_, const ImpedanceParam& param)
{
  i_param_.base_name = param.base_name.c_str();
  i_param_.target_name = param.target_name.c_str();
  i_param_.M_p = param.M_p;
  i_param_.D_p = param.D_p;
  i_param_.K_p = param.K_p;
  i_param_.M_r = param.M_r;
  i_param_.D_r = param.D_r;
  i_param_.K_r = param.K_r;
  for (size_t i = 0; i < 3; i++) i_param_.force_gain[i] = param.force_gain(i,i);
  for (size_t i = 0; i < 3; i++) i_param_.moment_gain[i] = param.moment_gain(i,i);
  i_param_.sr_gain = param.sr_gain;
  i_param_.avoid_gain = param.avoid_gain;
  i_param_.reference_gain = param.reference_gain;
  i_param_.manipulability_limit = param.manipulability_limit;
}

void ImpedanceController::updateRootLinkPosRot (TimedOrientation3D tmprpy)
{
  hrp::Sensor *sensor = m_robot->sensor(hrp::Sensor::ACCELERATION, 0);
  hrp::Matrix33 tmpr;
  rats::rotm3times(tmpr, hrp::Matrix33(sensor->link->R*sensor->localR).transpose(), m_robot->rootLink()->R);
  rats::rotm3times(m_robot->rootLink()->R, hrp::rotFromRpy(tmprpy.data.r, tmprpy.data.p, tmprpy.data.y), tmpr);
}

bool ImpedanceController::getImpedanceControllerParam(const std::string& i_name_, ImpedanceControllerService::impedanceParam& i_param_)
{
  int force_id = -1;
  if ( !checkImpedanceNameValidity (force_id, i_name_) ) {
    return false;
  }
  if ( m_impedance_param.find(i_name_) == m_impedance_param.end() ) { // if impedance param of i_name_ is not found, return default impedance parameter ;; default parameter is specified ImpedanceParam struct's default constructer
    copyImpedanceParam(i_param_, ImpedanceParam());
  } else {
    copyImpedanceParam(i_param_, m_impedance_param[i_name_]);
  }
  return true;
}

bool ImpedanceController::checkImpedanceNameValidity (int& force_id, const std::string& name)
{
  // error check
  force_id = -1;
  for (unsigned int i=0; i<m_forceIn.size(); i++){
    if ( std::string(m_forceIn[i]->name()) == name ) {
      force_id = i;
      break;
    }
  }
  if ( force_id < 0 ) {
    std::cerr << "[" << m_profile.instance_name << "] Could not found FORCE_SENSOR named " << name << std::endl;
    return false;
  }
  return true;
}

bool ImpedanceController::deleteImpedanceController(std::string i_name_)
{
    if ( m_impedance_param.find(i_name_) == m_impedance_param.end() ) {
      std::cerr << "[" << m_profile.instance_name << "] Could not found impedance controller" << i_name_ << std::endl;
      return false;
    }
     
    if ( m_impedance_param[i_name_].transition_count > 0) {
      std::cerr << "[" << m_profile.instance_name << "] " << i_name_ << "is already in deleting." << std::endl;
      return false;
    }else{
      std::cerr << "[" << m_profile.instance_name << "] Delete impedance parameters " << i_name_ << std::endl;
      for (int i = 0; i < m_robot->numJoints(); i++ ) {
          m_impedance_param[i_name_].transition_joint_q[i] = m_robot->joint(i)->q;
      }
      m_impedance_param[i_name_].transition_count = MAX_TRANSITION_COUNT; // when stop impedance, count down to 0
    }

    return true;
}

void ImpedanceController::waitDeletingImpedanceController(std::string i_name_)
{
    while (m_impedance_param.find(i_name_) != m_impedance_param.end() &&
           m_impedance_param[i_name_].transition_count > 0) {
      usleep(10);
    }
    return;
}

bool ImpedanceController::deleteImpedanceControllerAndWait(std::string i_name_)
{
    if(!deleteImpedanceController(i_name_)){
      return false;
    }

    // wait for transition count
    waitDeletingImpedanceController(i_name_);

    return true;
}

extern "C"
{

    void ImpedanceControllerInit(RTC::Manager* manager)
    {
        RTC::Properties profile(impedancecontroller_spec);
        manager->registerFactory(profile,
                                 RTC::Create<ImpedanceController>,
                                 RTC::Delete<ImpedanceController>);
    }

};


