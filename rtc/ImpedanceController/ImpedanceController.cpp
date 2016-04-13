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
#include "hrpsys/util/Hrpsys.h"
#include <boost/assign.hpp>

#define MAX_TRANSITION_COUNT (static_cast<int>(2/m_dt))
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
      m_basePosIn("basePosIn", m_basePos),
      m_baseRpyIn("baseRpyIn", m_baseRpy),
      m_rpyIn("rpy", m_rpy),
      m_qOut("q", m_q),
      m_ImpedanceControllerServicePort("ImpedanceControllerService"),
      // </rtc-template>
      m_robot(hrp::BodyPtr()),
      m_debugLevel(0),
      dummy(0),
      use_sh_base_pos_rpy(false)
{
    m_service0.impedance(this);
}

ImpedanceController::~ImpedanceController()
{
}


RTC::ReturnCode_t ImpedanceController::onInitialize()
{
    std::cerr << "[" << m_profile.instance_name << "] onInitialize()" << std::endl;
    bindParameter("debugLevel", m_debugLevel, "0");

    // Registration: InPort/OutPort/Service
    // <rtc-template block="registration">
    // Set InPort buffers
    addInPort("qCurrent", m_qCurrentIn);
    addInPort("qRef", m_qRefIn);
    addInPort("basePosIn", m_basePosIn);
    addInPort("baseRpyIn", m_baseRpyIn);
    addInPort("rpy", m_rpyIn);

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


    // Setting for wrench data ports (real + virtual)
    std::vector<std::string> fsensor_names;
    //   find names for real force sensors
    int npforce = m_robot->numSensors(hrp::Sensor::FORCE);
    for (unsigned int i=0; i<npforce; i++){
        fsensor_names.push_back(m_robot->sensor(hrp::Sensor::FORCE, i)->name);
    }
    // load virtual force sensors
    readVirtualForceSensorParamFromProperties(m_vfs, m_robot, prop["virtual_force_sensor"], std::string(m_profile.instance_name));
    int nvforce = m_vfs.size();
    for (unsigned int i=0; i<nvforce; i++){
        for ( std::map<std::string, hrp::VirtualForceSensorParam>::iterator it = m_vfs.begin(); it != m_vfs.end(); it++ ) {
            if (it->second.id == i) fsensor_names.push_back(it->first);
        }
    }
    //   add ports for all force sensors
    int nforce  = npforce + nvforce;
    m_force.resize(nforce);
    m_forceIn.resize(nforce);
    m_ref_force.resize(nforce);
    m_ref_forceIn.resize(nforce);
    std::cerr << "[" << m_profile.instance_name << "] force sensor ports" << std::endl;
    for (unsigned int i=0; i<nforce; i++){
        // actual inport
        m_forceIn[i] = new InPort<TimedDoubleSeq>(fsensor_names[i].c_str(), m_force[i]);
        m_force[i].data.length(6);
        registerInPort(fsensor_names[i].c_str(), *m_forceIn[i]);
        // ref inport
        m_ref_force[i].data.length(6);
        for (unsigned int j=0; j<6; j++) m_ref_force[i].data[j] = 0.0;
        m_ref_forceIn[i] = new InPort<TimedDoubleSeq>(std::string("ref_"+fsensor_names[i]+"In").c_str(), m_ref_force[i]);
        registerInPort(std::string("ref_"+fsensor_names[i]+"In").c_str(), *m_ref_forceIn[i]);
        std::cerr << "[" << m_profile.instance_name << "]   name = " << fsensor_names[i] << std::endl;
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

    // setting from conf file
    // rleg,TARGET_LINK,BASE_LINK,x,y,z,rx,ry,rz,rth #<=pos + rot (axis+angle)
    coil::vstring end_effectors_str = coil::split(prop["end_effectors"], ",");
    std::map<std::string, std::string> base_name_map;
    if (end_effectors_str.size() > 0) {
        size_t prop_num = 10;
        size_t num = end_effectors_str.size()/prop_num;
        for (size_t i = 0; i < num; i++) {
            std::string ee_name, ee_target, ee_base;
            coil::stringTo(ee_name, end_effectors_str[i*prop_num].c_str());
            coil::stringTo(ee_target, end_effectors_str[i*prop_num+1].c_str());
            coil::stringTo(ee_base, end_effectors_str[i*prop_num+2].c_str());
            ee_trans eet;
            for (size_t j = 0; j < 3; j++) {
                coil::stringTo(eet.localPos(j), end_effectors_str[i*prop_num+3+j].c_str());
            }
            double tmpv[4];
            for (int j = 0; j < 4; j++ ) {
                coil::stringTo(tmpv[j], end_effectors_str[i*prop_num+6+j].c_str());
            }
            eet.localR = Eigen::AngleAxis<double>(tmpv[3], hrp::Vector3(tmpv[0], tmpv[1], tmpv[2])).toRotationMatrix(); // rotation in VRML is represented by axis + angle
            eet.target_name = ee_target;
            ee_map.insert(std::pair<std::string, ee_trans>(ee_name , eet));
            base_name_map.insert(std::pair<std::string, std::string>(ee_name, ee_base));
            std::cerr << "[" << m_profile.instance_name << "] End Effector [" << ee_name << "]" << ee_target << " " << ee_base << std::endl;
            std::cerr << "[" << m_profile.instance_name << "]   target = " << ee_target << ", base = " << ee_base << std::endl;
            std::cerr << "[" << m_profile.instance_name << "]   localPos = " << eet.localPos.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "    [", "]")) << "[m]" << std::endl;
            std::cerr << "[" << m_profile.instance_name << "]   localR = " << eet.localR.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", "\n", "    [", "]")) << std::endl;
        }
    }

    // initialize impedance params
    std::cerr << "[" << m_profile.instance_name << "] Add impedance params" << std::endl;
    for (unsigned int i=0; i<m_forceIn.size(); i++){
        std::string sensor_name = m_forceIn[i]->name();
        hrp::ForceSensor* sensor = m_robot->sensor<hrp::ForceSensor>(sensor_name);
        std::string sensor_link_name;
        if ( sensor ) {
            // real force sensor
            sensor_link_name = sensor->link->name;
        } else if ( m_vfs.find(sensor_name) !=  m_vfs.end()) {
            // virtual force sensor
            sensor_link_name = m_vfs[sensor_name].link->name;
        } else {
            std::cerr << "[" << m_profile.instance_name << "]   unknown force param" << std::endl;
            continue;
        }
        // 1. Check whether adequate ee_map exists for the sensor.
        std::string ee_name;
        bool is_ee_exists = false;
        for ( std::map<std::string, ee_trans>::iterator it = ee_map.begin(); it != ee_map.end(); it++ ) {
            hrp::Link* alink = m_robot->link(it->second.target_name);
            std::string tmp_base_name = base_name_map[it->first];
            while (alink != NULL && alink->name != tmp_base_name && !is_ee_exists) {
                if ( alink->name == sensor_link_name ) {
                    is_ee_exists = true;
                    ee_name = it->first;
                }
                alink = alink->parent;
            }
        }
        if (!is_ee_exists) {
            std::cerr << "[" << m_profile.instance_name << "]   No such ee setting for " << sensor_name << " and " << sensor_link_name << "!!. Impedance param for " << sensor_name << " cannot be added!!" << std::endl;
            continue;
        }
        // 2. Check whether already impedance param exists, which has the same target link as the sensor.
        if (m_impedance_param.find(ee_name) != m_impedance_param.end()) {
            std::cerr << "[" << m_profile.instance_name << "]   Already impedance param (target_name=" << sensor_link_name << ", ee_name=" << ee_name << ") exists!!. Impedance param for " << sensor_name << " cannot be added!!" << std::endl;
            continue;
        }
        // 3. Check whether joint path is adequate.
        hrp::Link* target_link = m_robot->link(ee_map[ee_name].target_name);
        ImpedanceParam p;
        p.manip = hrp::JointPathExPtr(new hrp::JointPathEx(m_robot, m_robot->link(base_name_map[ee_name]), target_link, m_dt, false, std::string(m_profile.instance_name)));
        if ( ! p.manip ) {
            std::cerr << "[" << m_profile.instance_name << "]   Invalid joint path from " << base_name_map[ee_name] << " to " << target_link->name << "!! Impedance param for " << sensor_name << " cannot be added!!" << std::endl;
            continue;
        }
        // 4. Set impedance param
        p.transition_joint_q.resize(m_robot->numJoints());
        p.sensor_name = sensor_name;
        m_impedance_param[ee_name] = p;
        std::cerr << "[" << m_profile.instance_name << "]   sensor = " << sensor_name << ", sensor-link = " << sensor_link_name << ", ee_name = " << ee_name << ", ee-link = " << target_link->name << std::endl;
    }

    std::vector<std::pair<hrp::Link*, hrp::Link*> > interlocking_joints;
    readInterlockingJointsParamFromProperties(interlocking_joints, m_robot, prop["interlocking_joints"], std::string(m_profile.instance_name));
    if (interlocking_joints.size() > 0) {
        for ( std::map<std::string, ImpedanceParam>::iterator it = m_impedance_param.begin(); it != m_impedance_param.end(); it++ ) {
            std::cerr << "[" << m_profile.instance_name << "] Interlocking Joints for [" << it->first << "]" << std::endl;
            it->second.manip->setInterlockingJointPairIndices(interlocking_joints, std::string(m_profile.instance_name));
        }
    }

    otd = boost::shared_ptr<ObjectTurnaroundDetector>(new ObjectTurnaroundDetector(m_dt));
    otd->setPrintStr(std::string(m_profile.instance_name));

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
    std::cerr << "[" << m_profile.instance_name<< "] onActivated(" << ec_id << ")" << std::endl;
    
    return RTC::RTC_OK;
}

RTC::ReturnCode_t ImpedanceController::onDeactivated(RTC::UniqueId ec_id)
{
  std::cerr << "[" << m_profile.instance_name<< "] onDeactivated(" << ec_id << ")" << std::endl;
  for ( std::map<std::string, ImpedanceParam>::iterator it = m_impedance_param.begin(); it != m_impedance_param.end(); it++ ) {
      if (it->second.is_active) {
          stopImpedanceControllerNoWait(it->first);
          it->second.transition_count = 1;
      }
  }
  return RTC::RTC_OK;
}

#define DEBUGP ((m_debugLevel==1 && loop%200==0) || m_debugLevel > 1 )
RTC::ReturnCode_t ImpedanceController::onExecute(RTC::UniqueId ec_id)
{
    //std::cout << "ImpedanceController::onExecute(" << ec_id << ")" << std::endl;
    loop ++;

    // check dataport input
    for (unsigned int i=0; i<m_forceIn.size(); i++){
        if ( m_forceIn[i]->isNew() ) {
            m_forceIn[i]->read();
        }
        if ( m_ref_forceIn[i]->isNew() ) {
            m_ref_forceIn[i]->read();
        }
    }
    if (m_basePosIn.isNew()) {
      m_basePosIn.read();
    }
    if (m_baseRpyIn.isNew()) {
      m_baseRpyIn.read();
    }
    if (m_rpyIn.isNew()) {
      m_rpyIn.read();
    }
    if (m_qCurrentIn.isNew()) {
      m_qCurrentIn.read();
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

        Guard guard(m_mutex);

        bool is_active = false;
        for ( std::map<std::string, ImpedanceParam>::iterator it = m_impedance_param.begin(); it != m_impedance_param.end(); it++ ) {
            is_active = is_active || it->second.is_active;
        }
        if ( !is_active ) {
          for ( int i = 0; i < m_qRef.data.length(); i++ ){
            m_q.data[i] = m_qRef.data[i];
            m_robot->joint(i)->q = m_qRef.data[i];
          }
          m_qOut.write();
          return RTC_OK;
        }

	{
	  hrp::dvector qorg(m_robot->numJoints());
	  
	  // reference model
	  for ( int i = 0; i < m_robot->numJoints(); i++ ){
	    qorg[i] = m_robot->joint(i)->q;
            m_robot->joint(i)->q = m_qRef.data[i];
            qrefv[i] = m_qRef.data[i];
	  }
          m_robot->rootLink()->p = hrp::Vector3(m_basePos.data.x, m_basePos.data.y, m_basePos.data.z);
          m_robot->rootLink()->R = hrp::rotFromRpy(m_baseRpy.data.r, m_baseRpy.data.p, m_baseRpy.data.y);
          m_robot->calcForwardKinematics();
          if ( (ee_map.find("rleg") != ee_map.end() && ee_map.find("lleg") != ee_map.end()) // if legged robot
               && !use_sh_base_pos_rpy ) {
              // TODO
              //  Tempolarily modify root coords to fix foot pos rot
              //  This will be removed after seq outputs adequate waistRPY discussed in https://github.com/fkanehiro/hrpsys-base/issues/272

              // get current foot mid pos + rot
              std::vector<hrp::Vector3> foot_pos;
              std::vector<hrp::Matrix33> foot_rot;
              std::vector<std::string> leg_names;
              leg_names.push_back("rleg");
              leg_names.push_back("lleg");
              for (size_t i = 0; i < leg_names.size(); i++) {
                  hrp::Link* target_link = m_robot->link(ee_map[leg_names[i]].target_name);
                  foot_pos.push_back(target_link->p + target_link->R * ee_map[leg_names[i]].localPos);
                  foot_rot.push_back(target_link->R * ee_map[leg_names[i]].localR);
              }
              hrp::Vector3 current_foot_mid_pos ((foot_pos[0]+foot_pos[1])/2.0);
              hrp::Matrix33 current_foot_mid_rot;
              rats::mid_rot(current_foot_mid_rot, 0.5, foot_rot[0], foot_rot[1]);
              // calculate fix pos + rot
              hrp::Vector3 new_foot_mid_pos(current_foot_mid_pos);
              hrp::Matrix33 new_foot_mid_rot;
              {
                  hrp::Vector3 ex = hrp::Vector3::UnitX();
                  hrp::Vector3 ez = hrp::Vector3::UnitZ();
                  hrp::Vector3 xv1 (current_foot_mid_rot * ex);
                  xv1(2) = 0.0;
                  xv1.normalize();
                  hrp::Vector3 yv1(ez.cross(xv1));
                  new_foot_mid_rot(0,0) = xv1(0); new_foot_mid_rot(1,0) = xv1(1); new_foot_mid_rot(2,0) = xv1(2);
                  new_foot_mid_rot(0,1) = yv1(0); new_foot_mid_rot(1,1) = yv1(1); new_foot_mid_rot(2,1) = yv1(2);
                  new_foot_mid_rot(0,2) = ez(0); new_foot_mid_rot(1,2) = ez(1); new_foot_mid_rot(2,2) = ez(2);
              }
              // fix root pos + rot to fix "coords" = "current_foot_mid_xx"
              hrp::Matrix33 tmpR (new_foot_mid_rot * current_foot_mid_rot.transpose());
              m_robot->rootLink()->p = new_foot_mid_pos + tmpR * (m_robot->rootLink()->p - current_foot_mid_pos);
              rats::rotm3times(m_robot->rootLink()->R, tmpR, m_robot->rootLink()->R);
              m_robot->calcForwardKinematics();
          }
          calcForceMoment();

	  // set sequencer position to target_p0
	  for ( std::map<std::string, ImpedanceParam>::iterator it = m_impedance_param.begin(); it != m_impedance_param.end(); it++ ) {
            ImpedanceParam& param = it->second;
            std::string target_name = ee_map[it->first].target_name;
            param.target_p0 = m_robot->link(target_name)->p + m_robot->link(target_name)->R * ee_map[it->first].localPos;
            param.target_r0 = m_robot->link(target_name)->R * ee_map[it->first].localR;
            if (param.transition_count == -MAX_TRANSITION_COUNT) param.resetPreviousTargetParam();
          }
          // back to impedance robot model (only for controlled joint)
	  for ( std::map<std::string, ImpedanceParam>::iterator it = m_impedance_param.begin(); it != m_impedance_param.end(); it++ ) {
            ImpedanceParam& param = it->second;
            if (param.is_active) {
                for ( int j = 0; j < param.manip->numJoints(); j++ ){
                    int i = param.manip->joint(j)->jointId;
                    m_robot->joint(i)->q = qorg[i];
                }
            }
	  }
	  m_robot->calcForwardKinematics();

	}

	// set m_robot to qRef when deleting status
        std::map<std::string, ImpedanceParam>::iterator it = m_impedance_param.begin();
	while(it != m_impedance_param.end()){
            ImpedanceParam& param = it->second;
            if (param.is_active) {
                if (DEBUGP) {
                  std::cerr << "[" << m_profile.instance_name << "] impedance mode " << it->first << " transition count = " << param.transition_count << ", ";
                  std::cerr << "MDK = " << param.M_p << " " << param.D_p << " " << param.K_p << ", ";
                  std::cerr << "MDK = " << param.M_r << " " << param.D_r << " " << param.K_r << ", ";
                  std::cerr << "ref_force = " << param.ref_force[0] << " " << param.ref_force[1] << " " << param.ref_force[2] << ", ";
                  std::cerr << "ref_moment = " << param.ref_moment[0] << " " << param.ref_moment[1] << " " << param.ref_moment[2] << std::endl;
                }
                if ( param.transition_count > 0 ) {
                    hrp::JointPathExPtr manip = param.manip;
                    for ( int j = 0; j < manip->numJoints(); j++ ) {
                        int i = manip->joint(j)->jointId; // index in robot model
                        hrp::Link* joint =  m_robot->joint(i);
                        // transition_smooth_gain moves from 0 to 1
                        // (/ (log (/ (- 1 0.99) 0.99)) 0.5)
                        double transition_smooth_gain = 1/(1+exp(-9.19*((static_cast<double>(MAX_TRANSITION_COUNT - param.transition_count) / MAX_TRANSITION_COUNT) - 0.5)));
                        joint->q = ( m_qRef.data[i] - param.transition_joint_q[i] ) * transition_smooth_gain + param.transition_joint_q[i];
                    }
                    param.transition_count--;
                    if(param.transition_count <= 0){ // erase impedance param
                        std::cerr << "[" << m_profile.instance_name << "] Finished cleanup and erase impedance param " << it->first << std::endl;
                        param.is_active = false;
                    }
                } else {
                    // use impedance model

                    hrp::Link* target = m_robot->link(ee_map[it->first].target_name);
                    assert(target);
                    param.current_p1 = target->p + target->R * ee_map[it->first].localPos;
                    param.current_r1 = target->R * ee_map[it->first].localR;
                    if (param.transition_count == -MAX_TRANSITION_COUNT) param.resetPreviousCurrentParam();

                    hrp::Vector3 vel_p, vel_r;
                    //std::cerr << "MDK = " << param.M_p << " " << param.D_p << " " << param.K_p << std::endl;
                    //std::cerr << "MDK = " << param.M_r << " " << param.D_r << " " << param.K_r << std::endl;
                    // std::cerr << "ref_force = " << param.ref_force[0] << " " << param.ref_force[1] << " " << param.ref_force[2] << std::endl;
                    // std::cerr << "ref_moment = " << param.ref_moment[0] << " " << param.ref_moment[1] << " " << param.ref_moment[2] << std::endl;

                    // ref_force/ref_moment and force_gain/moment_gain are expressed in global coordinates. 
                    hrp::Matrix33 eeR = target->R * ee_map[it->first].localR;
                    hrp::Vector3 force_diff = abs_forces[it->second.sensor_name] - abs_ref_forces[it->second.sensor_name];
                    hrp::Vector3 moment_diff = abs_moments[it->second.sensor_name] - abs_ref_moments[it->second.sensor_name];
                    param.calcTargetVelocity(vel_p, vel_r, eeR, force_diff, moment_diff, m_dt,
                                             DEBUGP, std::string(m_profile.instance_name), it->first);

                    // Solve ik
                    hrp::JointPathExPtr manip = param.manip;
                    assert(manip);
                    //const int n = manip->numJoints();
                    manip->calcInverseKinematics2Loop(param.getOutputPos(), param.getOutputRot(), 1.0, param.avoid_gain, param.reference_gain, &qrefv, 1.0,
                                                      ee_map[it->first].localPos, ee_map[it->first].localR);

                    if ( param.transition_count < 0 ) {
                        param.transition_count++;
                    }
                } // else
            }
            it++;
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

        if (ee_map.find("rleg") != ee_map.end() && ee_map.find("lleg") != ee_map.end()) // if legged robot
            calcObjectTurnaroundDetectorState();
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

void ImpedanceController::calcFootMidCoords (hrp::Vector3& new_foot_mid_pos, hrp::Matrix33& new_foot_mid_rot)
{
  std::vector<hrp::Vector3> foot_pos;
  std::vector<hrp::Matrix33> foot_rot;
  std::vector<std::string> leg_names = boost::assign::list_of("rleg")("lleg");
  for (size_t i = 0; i < leg_names.size(); i++) {
    hrp::Link* target_link = m_robot->link(ee_map[leg_names[i]].target_name);
    foot_pos.push_back(target_link->p + target_link->R * ee_map[leg_names[i]].localPos);
    foot_rot.push_back(target_link->R * ee_map[leg_names[i]].localR);
  }
  new_foot_mid_pos = (foot_pos[0]+foot_pos[1])/2.0;
  rats::mid_rot(new_foot_mid_rot, 0.5, foot_rot[0], foot_rot[1]);
}

void ImpedanceController::calcForceMoment ()
{
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
          hrp::Vector3 sensorPos, eePos;
          if ( sensor ) {
            // real force sensore
            sensorR = sensor->link->R * sensor->localR;
            sensorPos = sensor->link->p + sensorR * sensor->localPos;
          } else if ( m_vfs.find(sensor_name) !=  m_vfs.end()) {
            // virtual force sensor
            if ( DEBUGP ) {
              std::cerr << "[" << m_profile.instance_name << "]   sensorR = " << m_vfs[sensor_name].localR.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", "\n", "    [", "]")) << std::endl;
            }
            sensorR = m_vfs[sensor_name].link->R * m_vfs[sensor_name].localR;
            sensorPos = m_vfs[sensor_name].link->p + m_vfs[sensor_name].link->R * m_vfs[sensor_name].localPos;
          } else {
            std::cerr << "[" << m_profile.instance_name << "]   unknown force param" << std::endl;
          }
          abs_forces[sensor_name] = sensorR * data_p;
          for ( std::map<std::string, ImpedanceParam>::iterator it = m_impedance_param.begin(); it != m_impedance_param.end(); it++ ) {
              if ( it->second.sensor_name == sensor_name ) {
                  std::string ee_name = it->first;
                  hrp::Link* target_link = m_robot->link(ee_map[ee_name].target_name);
                  eePos = target_link->p + target_link->R * ee_map[ee_name].localPos;
              }
          }
          abs_moments[sensor_name] = sensorR * data_r + (sensorPos - eePos).cross(abs_forces[sensor_name]);
          // End effector local frame
          // hrp::Matrix33 eeR (parentlink->R * ee_map[parentlink->name].localR);
          // abs_ref_forces[sensor_name] = eeR * ref_data_p;
          // abs_ref_moments[sensor_name] = eeR * ref_data_r;
          // World frame
          abs_ref_forces[sensor_name] = ref_data_p;
          abs_ref_moments[sensor_name] = ref_data_r;
          if ( DEBUGP ) {
            std::cerr << "[" << m_profile.instance_name << "]   abs force  = " << abs_forces[sensor_name].format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[N]" << std::endl;
            std::cerr << "[" << m_profile.instance_name << "]   abs moment = " << abs_moments[sensor_name].format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[Nm]" << std::endl;
            std::cerr << "[" << m_profile.instance_name << "]   abs ref force  = " << abs_ref_forces[sensor_name].format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[N]" << std::endl;
            std::cerr << "[" << m_profile.instance_name << "]   abs ref moment = " << abs_ref_moments[sensor_name].format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[Nm]" << std::endl;
          }
        }
      }
};

void ImpedanceController::calcObjectTurnaroundDetectorState()
{
    // TODO
    // Currently only for legged robots
    // Store org state
    hrp::dvector org_q(m_robot->numJoints());
    for ( int i = 0; i < m_robot->numJoints(); i++ ) {
        org_q[i] = m_robot->joint(i)->q;
    }
    hrp::Matrix33 orgR = m_robot->rootLink()->R;
    // Set actual state
    for ( int i = 0; i < m_robot->numJoints(); i++ ) {
        m_robot->joint(i)->q = m_qCurrent.data[i];
    }
    updateRootLinkPosRot(m_rpy);
    // Calc
    std::vector<hrp::Vector3> otd_fmv, otd_hposv;
    hrp::Vector3 fmpos;
    hrp::Matrix33 fmrot, fmrotT;
    calcFootMidCoords(fmpos, fmrot);
    fmrotT = fmrot.transpose();
    for (unsigned int i=0; i<m_forceIn.size(); i++) {
        std::string sensor_name = m_forceIn[i]->name();
        if ( find(otd_sensor_names.begin(), otd_sensor_names.end(), sensor_name) != otd_sensor_names.end() ) {
            hrp::ForceSensor* sensor = m_robot->sensor<hrp::ForceSensor>(sensor_name);
            hrp::Vector3 data_p(m_force[i].data[0], m_force[i].data[1], m_force[i].data[2]);
            hrp::Vector3 data_r(m_force[i].data[3], m_force[i].data[4], m_force[i].data[5]);
            hrp::Matrix33 sensorR = sensor->link->R * sensor->localR;
            otd_fmv.push_back(fmrotT*(sensorR*data_p));
            hrp::Vector3 eePos;
            for ( std::map<std::string, ImpedanceParam>::iterator it = m_impedance_param.begin(); it != m_impedance_param.end(); it++ ) {
                if ( it->second.sensor_name == sensor_name ) {
                    ee_trans& eet = ee_map[it->first];
                    hrp::Link* target_link = m_robot->link(eet.target_name);
                    eePos = fmrotT * (target_link->p + target_link->R * eet.localPos - fmpos);
                }
            }
            otd_hposv.push_back(eePos);
        }
    }
    otd->checkDetection(otd_fmv, otd_hposv);
    // Revert to org state
    for ( int i = 0; i < m_robot->numJoints(); i++ ) {
        m_robot->joint(i)->q = org_q[i];
    }
    m_robot->rootLink()->R = orgR;
};

//
bool ImpedanceController::startImpedanceControllerNoWait(const std::string& i_name_)
{
    // Lock Mutex
    {
        Guard guard(m_mutex);
        if ( m_impedance_param.find(i_name_) == m_impedance_param.end() ) {
            std::cerr << "[" << m_profile.instance_name << "] Could not found impedance controller param [" << i_name_ << "]" << std::endl;
            return false;
        }
        if ( m_impedance_param[i_name_].is_active ) {
            std::cerr << "[" << m_profile.instance_name << "] Impedance control [" << i_name_ << "] is already started" << std::endl;
            return false;
        }
        std::cerr << "[" << m_profile.instance_name << "] Start impedance control [" << i_name_ << "]" << std::endl;
        m_impedance_param[i_name_].is_active = true;
        m_impedance_param[i_name_].transition_count = -MAX_TRANSITION_COUNT; // when start impedance, count up to 0
    }
    return true;
}

 bool ImpedanceController::startImpedanceController(const std::string& i_name_)
 {
     bool ret = startImpedanceControllerNoWait(i_name_);
     waitImpedanceControllerTransition(i_name_);
     return ret;
 }

bool ImpedanceController::stopImpedanceControllerNoWait(const std::string& i_name_)
{
    // Lock Mutex
    {
        Guard guard(m_mutex);
        if ( m_impedance_param.find(i_name_) == m_impedance_param.end() ) {
            std::cerr << "[" << m_profile.instance_name << "] Could not found impedance controller param [" << i_name_ << "]" << std::endl;
            return false;
        }
        if ( !m_impedance_param[i_name_].is_active ) {
            std::cerr << "[" << m_profile.instance_name << "] Impedance control [" << i_name_ << "] is already stopped" << std::endl;
            return false;
        }
        std::cerr << "[" << m_profile.instance_name << "] Stop impedance control [" << i_name_ << "]" << std::endl;
        for (int i = 0; i < m_robot->numJoints(); i++ ) {
            m_impedance_param[i_name_].transition_joint_q[i] = m_robot->joint(i)->q;
        }
        m_impedance_param[i_name_].transition_count = MAX_TRANSITION_COUNT; // when stop impedance, count down to 0
    }
    return true;
}

bool ImpedanceController::stopImpedanceController(const std::string& i_name_)
{
    bool ret = stopImpedanceControllerNoWait(i_name_);
    waitImpedanceControllerTransition(i_name_);
    return ret;
}

bool ImpedanceController::setImpedanceControllerParam(const std::string& i_name_, OpenHRP::ImpedanceControllerService::impedanceParam i_param_)
{
    // Lock Mutex
    {
        Guard guard(m_mutex);
        std::string name = std::string(i_name_);
        if ( m_impedance_param.find(name) == m_impedance_param.end() ) {
            std::cerr << "[" << m_profile.instance_name << "] Could not found impedance controller param [" << name << "]" << std::endl;
            return false;
        }

        std::cerr << "[" << m_profile.instance_name << "] Update impedance parameters" << std::endl;

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

        std::vector<double> ov;
        ov.resize(m_impedance_param[name].manip->numJoints());
        for (size_t i = 0; i < m_impedance_param[name].manip->numJoints(); i++) {
            ov[i] = i_param_.ik_optional_weight_vector[i];
        }
        m_impedance_param[name].manip->setOptionalWeightVector(ov);
        use_sh_base_pos_rpy = i_param_.use_sh_base_pos_rpy;

        std::cerr << "[" << m_profile.instance_name << "] set parameters" << std::endl;
        std::cerr << "[" << m_profile.instance_name << "]             name : " << name << std::endl;
        std::cerr << "[" << m_profile.instance_name << "]    M, D, K (pos) : " << m_impedance_param[name].M_p << " " << m_impedance_param[name].D_p << " " << m_impedance_param[name].K_p << std::endl;
        std::cerr << "[" << m_profile.instance_name << "]    M, D, K (rot) : " << m_impedance_param[name].M_r << " " << m_impedance_param[name].D_r << " " << m_impedance_param[name].K_r << std::endl;
        std::cerr << "[" << m_profile.instance_name << "]       force_gain : " << m_impedance_param[name].force_gain.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", "\n", "    [", "]")) << std::endl;
        std::cerr << "[" << m_profile.instance_name << "]      moment_gain : " << m_impedance_param[name].moment_gain.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", "\n", "    [", "]")) << std::endl;
        std::cerr << "[" << m_profile.instance_name << "]      manip_limit : " << m_impedance_param[name].manipulability_limit << std::endl;
        std::cerr << "[" << m_profile.instance_name << "]          sr_gain : " << m_impedance_param[name].sr_gain << std::endl;
        std::cerr << "[" << m_profile.instance_name << "]       avoid_gain : " << m_impedance_param[name].avoid_gain << std::endl;
        std::cerr << "[" << m_profile.instance_name << "]   reference_gain : " << m_impedance_param[name].reference_gain << std::endl;
        std::cerr << "[" << m_profile.instance_name << "]   use_sh_base_pos_rpy : " << (use_sh_base_pos_rpy?"true":"false") << std::endl;
    }
    return true;
}

void ImpedanceController::copyImpedanceParam (ImpedanceControllerService::impedanceParam& i_param_, const ImpedanceParam& param)
{
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
  if (param.is_active) i_param_.controller_mode = OpenHRP::ImpedanceControllerService::MODE_IMP;
  else i_param_.controller_mode = OpenHRP::ImpedanceControllerService::MODE_IDLE;
  if (param.manip == NULL) i_param_.ik_optional_weight_vector.length(0);
  else {
      i_param_.ik_optional_weight_vector.length(param.manip->numJoints());
      std::vector<double> ov;
      ov.resize(param.manip->numJoints());
      param.manip->getOptionalWeightVector(ov);
      for (size_t i = 0; i < param.manip->numJoints(); i++) {
          i_param_.ik_optional_weight_vector[i] = ov[i];
      }
  }
}

void ImpedanceController::updateRootLinkPosRot (TimedOrientation3D tmprpy)
{
  if ( m_robot->numSensors(hrp::Sensor::ACCELERATION) > 0) {
      hrp::Sensor *sensor = m_robot->sensor(hrp::Sensor::ACCELERATION, 0);
      hrp::Matrix33 tmpr;
      rats::rotm3times(tmpr, hrp::Matrix33(sensor->link->R*sensor->localR).transpose(), m_robot->rootLink()->R);
      rats::rotm3times(m_robot->rootLink()->R, hrp::rotFromRpy(tmprpy.data.r, tmprpy.data.p, tmprpy.data.y), tmpr);
  }
}

bool ImpedanceController::getImpedanceControllerParam(const std::string& i_name_, ImpedanceControllerService::impedanceParam& i_param_)
{
    if ( m_impedance_param.find(i_name_) == m_impedance_param.end() ) {
        std::cerr << "[" << m_profile.instance_name << "] Could not found impedance controller param [" << i_name_ << "]" << std::endl;
        // if impedance param of i_name_ is not found, return default impedance parameter ;; default parameter is specified ImpedanceParam struct's default constructer
        copyImpedanceParam(i_param_, ImpedanceParam());
        i_param_.use_sh_base_pos_rpy = use_sh_base_pos_rpy;
        return false;
    }
    copyImpedanceParam(i_param_, m_impedance_param[i_name_]);
    i_param_.use_sh_base_pos_rpy = use_sh_base_pos_rpy;
    return true;
}

void ImpedanceController::waitImpedanceControllerTransition(std::string i_name_)
{
    while (m_impedance_param.find(i_name_) != m_impedance_param.end() &&
           m_impedance_param[i_name_].transition_count != 0) {
      usleep(10);
    }
    return;
}

//
// ObjectTurnaroundDetector
//

void ImpedanceController::startObjectTurnaroundDetection(const double i_ref_diff_wrench, const double i_max_time, const OpenHRP::ImpedanceControllerService::StrSequence& i_ee_names)
{
    otd->startDetection(i_ref_diff_wrench, i_max_time);
    otd_sensor_names.clear();
    for (size_t i = 0; i < i_ee_names.length(); i++) {
        otd_sensor_names.push_back(m_impedance_param[std::string(i_ee_names[i])].sensor_name);
    }
}

OpenHRP::ImpedanceControllerService::DetectorMode ImpedanceController::checkObjectTurnaroundDetection()
{
    OpenHRP::ImpedanceControllerService::DetectorMode tmpmode;
    switch (otd->getMode()) {
    case ObjectTurnaroundDetector::MODE_IDLE:
        tmpmode = ImpedanceControllerService::MODE_DETECTOR_IDLE;
        break;
    case ObjectTurnaroundDetector::MODE_STARTED:
        tmpmode = ImpedanceControllerService::MODE_STARTED;
        break;
    case ObjectTurnaroundDetector::MODE_DETECTED:
        tmpmode = ImpedanceControllerService::MODE_DETECTED;
        break;
    case ObjectTurnaroundDetector::MODE_MAX_TIME:
        tmpmode = ImpedanceControllerService::MODE_MAX_TIME;
        break;
    default:
        tmpmode = ImpedanceControllerService::MODE_DETECTOR_IDLE;
        break;
    }
    return tmpmode;
}

bool ImpedanceController::setObjectTurnaroundDetectorParam(const OpenHRP::ImpedanceControllerService::objectTurnaroundDetectorParam &i_param_)
{
    std::cerr << "[" << m_profile.instance_name << "] setObjectTurnaroundDetectorParam" << std::endl;
    Guard guard(m_mutex);
    otd->setWrenchCutoffFreq(i_param_.wrench_cutoff_freq);
    otd->setDwrenchCutoffFreq(i_param_.dwrench_cutoff_freq);
    otd->setDetectRatioThre(i_param_.detect_ratio_thre);
    otd->setStartRatioThre(i_param_.start_ratio_thre);
    otd->setDetectTimeThre(i_param_.detect_time_thre);
    otd->setStartTimeThre(i_param_.start_time_thre);
    hrp::Vector3 tmp;
    for (size_t i = 0; i < 3; i++) tmp(i) = i_param_.axis[i];
    otd->setAxis(tmp);
    for (size_t i = 0; i < 3; i++) tmp(i) = i_param_.moment_center[i];
    otd->setMomentCenter(tmp);
    otd->setDetectorTotalWrench((i_param_.detector_total_wrench==OpenHRP::ImpedanceControllerService::TOTAL_FORCE)?ObjectTurnaroundDetector::TOTAL_FORCE:ObjectTurnaroundDetector::TOTAL_MOMENT);
    otd->printParams();
    return true;
};

bool ImpedanceController::getObjectTurnaroundDetectorParam(OpenHRP::ImpedanceControllerService::objectTurnaroundDetectorParam& i_param_)
{
    std::cerr << "[" << m_profile.instance_name << "] getObjectTurnaroundDetectorParam" << std::endl;
    i_param_.wrench_cutoff_freq = otd->getWrenchCutoffFreq();
    i_param_.dwrench_cutoff_freq = otd->getDwrenchCutoffFreq();
    i_param_.detect_ratio_thre = otd->getDetectRatioThre();
    i_param_.start_ratio_thre = otd->getStartRatioThre();
    i_param_.detect_time_thre = otd->getDetectTimeThre();
    i_param_.start_time_thre = otd->getStartTimeThre();
    hrp::Vector3 tmp = otd->getAxis();
    for (size_t i = 0; i < 3; i++) i_param_.axis[i] = tmp(i);
    tmp = otd->getMomentCenter();
    for (size_t i = 0; i < 3; i++) i_param_.moment_center[i] = tmp(i);
    i_param_.detector_total_wrench = (otd->getDetectorTotalWrench()==ObjectTurnaroundDetector::TOTAL_FORCE)?OpenHRP::ImpedanceControllerService::TOTAL_FORCE:OpenHRP::ImpedanceControllerService::TOTAL_MOMENT;
    return true;
}

bool ImpedanceController::getObjectForcesMoments(OpenHRP::ImpedanceControllerService::Dbl3Sequence_out o_forces, OpenHRP::ImpedanceControllerService::Dbl3Sequence_out o_moments, OpenHRP::ImpedanceControllerService::DblSequence3_out o_3dofwrench)
{
    std::cerr << "[" << m_profile.instance_name << "] getObjectForcesMoments" << std::endl;
    if (otd_sensor_names.size() == 0) return false;
    hrp::Vector3 tmpv = otd->getAxis() * otd->getFilteredWrench();
    o_forces = new OpenHRP::ImpedanceControllerService::Dbl3Sequence ();
    o_moments = new OpenHRP::ImpedanceControllerService::Dbl3Sequence ();
    o_forces->length(otd_sensor_names.size());
    o_moments->length(otd_sensor_names.size());
    for (size_t i = 0; i < o_forces->length(); i++) o_forces[i].length(3);
    for (size_t i = 0; i < o_moments->length(); i++) o_moments[i].length(3);
    // Temp
    for (size_t i = 0; i < otd_sensor_names.size(); i++) {
        o_forces[i][0] = tmpv(0)/otd_sensor_names.size();
        o_forces[i][1] = tmpv(1)/otd_sensor_names.size();
        o_forces[i][2] = tmpv(2)/otd_sensor_names.size();
        o_moments[i][0] = o_moments[i][1] = o_moments[i][2] = 0.0;
    }
    o_3dofwrench = new OpenHRP::ImpedanceControllerService::DblSequence3 ();
    o_3dofwrench->length(3);
    for (size_t i = 0; i < 3; i++) (*o_3dofwrench)[i] = tmpv(i);
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
