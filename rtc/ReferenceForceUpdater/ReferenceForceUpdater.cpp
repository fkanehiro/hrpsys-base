// -*- C++ -*-
/*!
 * @file  ReferenceForceUpdater.cpp
 * @brief Update ReferenceForce
 * $Date$
 *
 * $Id$
 */

#include <rtm/CorbaNaming.h>
#include <hrpModel/Link.h>
#include <hrpModel/Sensor.h>
#include <hrpModel/ModelLoaderUtil.h>
#include <hrpModel/JointPath.h>
#include <hrpUtil/MatrixSolvers.h>
#include "hrpsys/util/Hrpsys.h"
#include <boost/assign.hpp>
#include "ReferenceForceUpdater.h"
#include "hrpsys/util/VectorConvert.h"

typedef coil::Guard<coil::Mutex> Guard;

// Module specification
// <rtc-template block="module_spec">
static const char* ReferenceForceUpdater_spec[] =
  {
    "implementation_id", "ReferenceForceUpdater",
    "type_name",         "ReferenceForceUpdater",
    "description",       "update reference force",
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

ReferenceForceUpdater::ReferenceForceUpdater(RTC::Manager* manager)
  : RTC::DataFlowComponentBase(manager),
    // <rtc-template block="initializer">
    m_qRefIn("qRef", m_qRef),
    m_basePosIn("basePosIn", m_basePos),
    m_baseRpyIn("baseRpyIn", m_baseRpy),
    m_rpyIn("rpy", m_rpy),
    m_ReferenceForceUpdaterServicePort("ReferenceForceUpdaterService"),
    // </rtc-template>
    m_robot(hrp::BodyPtr()),
    m_debugLevel(0),
    use_sh_base_pos_rpy(false)
{
  m_ReferenceForceUpdaterService.rfu(this);
}

ReferenceForceUpdater::~ReferenceForceUpdater()
{
}



RTC::ReturnCode_t ReferenceForceUpdater::onInitialize()
{
  std::cerr << "[" << m_profile.instance_name << "] onInitialize()" << std::endl;
  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  bindParameter("debugLevel", m_debugLevel, "0");

  // </rtc-template>

  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("qRef", m_qRefIn);
  addInPort("basePosIn", m_basePosIn);
  addInPort("baseRpyIn",m_baseRpyIn);
  addInPort("rpy",m_rpyIn);

  // Set service provider to Ports
  m_ReferenceForceUpdaterServicePort.registerProvider("service0", "ReferenceForceUpdaterService", m_ReferenceForceUpdaterService);

  // Set service consumers to Ports
  // Set CORBA Service Ports
  addPort(m_ReferenceForceUpdaterServicePort);

  // Get dt
  RTC::Properties& prop = getProperties(); // get properties information from .wrl file
  coil::stringTo(m_dt, prop["dt"].c_str());

  // Make m_robot instance
  m_robot = hrp::BodyPtr(new hrp::Body());
  RTC::Manager& rtcManager = RTC::Manager::instance();
  std::string nameServer = rtcManager.getConfig()["corba.nameservers"];
  int comPos = nameServer.find(",");
  if (comPos < 0){
    comPos = nameServer.length();
  }
  nameServer = nameServer.substr(0, comPos);
  RTC::CorbaNaming naming(rtcManager.getORB(), nameServer.c_str());
  if (!loadBodyFromModelLoader(m_robot, prop["model"].c_str(), // load robot model for m_robot
                               CosNaming::NamingContext::_duplicate(naming.getRootContext())
                               )){
    std::cerr << "[" << m_profile.instance_name << "] failed to load model[" << prop["model"] << "]" << std::endl;
    return RTC::RTC_ERROR;
  }

  // Setting for wrench data ports (real + virtual)
  std::vector<std::string> fsensor_names;
  //   find names for real force sensors
  unsigned int npforce = m_robot->numSensors(hrp::Sensor::FORCE);
  for (unsigned int i=0; i<npforce; i++){
    fsensor_names.push_back(m_robot->sensor(hrp::Sensor::FORCE, i)->name);
  }
  // load virtual force sensors
  readVirtualForceSensorParamFromProperties(m_vfs, m_robot, prop["virtual_force_sensor"], std::string(m_profile.instance_name));
  unsigned int nvforce = m_vfs.size();
  for (unsigned int i=0; i<nvforce; i++){
    for ( std::map<std::string, hrp::VirtualForceSensorParam>::iterator it = m_vfs.begin(); it != m_vfs.end(); it++ ) {
      if (it->second.id == (int)i) fsensor_names.push_back(it->first);
    }
  }

  //   add ports for all force sensors (real force, input and output of ref_force)
  unsigned int nforce  = npforce + nvforce;
  m_force.resize(nforce);
  m_forceIn.resize(nforce);
  m_ref_force_in.resize(nforce);
  m_ref_force_out.resize(nforce);
  m_ref_forceIn.resize(nforce);
  m_ref_forceOut.resize(nforce);
  std::cerr << "[" << m_profile.instance_name << "] create force sensor ports" << std::endl;
  for (unsigned int i=0; i<nforce; i++){
    // actual inport
    m_forceIn[i] = new InPort<TimedDoubleSeq>(fsensor_names[i].c_str(), m_force[i]);
    m_force[i].data.length(6);
    registerInPort(fsensor_names[i].c_str(), *m_forceIn[i]);
    // ref inport
    m_ref_force_in[i].data.length(6);
    for (unsigned int j=0; j<6; j++) m_ref_force_in[i].data[j] = 0.0;
    m_ref_forceIn[i] = new InPort<TimedDoubleSeq>(std::string("ref_"+fsensor_names[i]+"In").c_str(), m_ref_force_in[i]);
    registerInPort(std::string("ref_"+fsensor_names[i]+"In").c_str(), *m_ref_forceIn[i]);
    std::cerr << "[" << m_profile.instance_name << "]   name = " << fsensor_names[i] << std::endl;
    // ref Outport
    m_ref_force_out[i].data.length(6);
    for (unsigned int j=0; j<6; j++) m_ref_force_out[i].data[j] = 0.0;
    m_ref_forceOut[i] = new OutPort<TimedDoubleSeq>(std::string("ref_"+fsensor_names[i]+"Out").c_str(), m_ref_force_out[i]);
    registerOutPort(std::string("ref_"+fsensor_names[i]+"Out").c_str(), *m_ref_forceOut[i]);
    std::cerr << "[" << m_profile.instance_name << "]   name = " << fsensor_names[i] << std::endl;
  }

  // setting from conf file
  // rleg,TARGET_LINK,BASE_LINK,x,y,z,rx,ry,rz,rth #<=pos + rot (axis+angle)
  coil::vstring end_effectors_str = coil::split(prop["end_effectors"], ",");
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

      ReferenceForceUpdaterParam rfu_param;
      //set rfu param
      rfu_param.update_count = round((1/rfu_param.update_freq)/m_dt);
      if (( ee_name != "rleg" ) && ( ee_name != "lleg" ))
        m_RFUParam.insert(std::pair<std::string, ReferenceForceUpdaterParam>(ee_name , rfu_param));

      ee_index_map.insert(std::pair<std::string, size_t>(ee_name, i));
      ref_force.push_back(hrp::Vector3::Zero());
      //ref_force_interpolator.insert(std::pair<std::string, interpolator*>(ee_name, new interpolator(3, m_dt)));
      ref_force_interpolator.insert(std::pair<std::string, interpolator*>(ee_name, new interpolator(3, m_dt, interpolator::LINEAR)));
      if (( ee_name != "lleg" ) && ( ee_name != "rleg" )) transition_interpolator.insert(std::pair<std::string, interpolator*>(ee_name, new interpolator(1, m_dt)));
      std::cerr << "[" << m_profile.instance_name << "] End Effector [" << ee_name << "]" << ee_target << " " << ee_base << std::endl;
      std::cerr << "[" << m_profile.instance_name << "]   target = " << ee_target << ", base = " << ee_base << std::endl;
      std::cerr << "[" << m_profile.instance_name << "]   localPos = " << eet.localPos.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "    [", "]")) << "[m]" << std::endl;
      std::cerr << "[" << m_profile.instance_name << "]   localR = " << eet.localR.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", "\n", "    [", "]")) << std::endl;
    }
  }

  // check if the dof of m_robot match the number of joint in m_robot
  unsigned int dof = m_robot->numJoints();
  for ( unsigned int i = 0 ; i < dof; i++ ){
    if ( (int)i != m_robot->joint(i)->jointId ) {
      std::cerr << "[" << m_profile.instance_name << "] jointId is not equal to the index number" << std::endl;
      return RTC::RTC_ERROR;
    }
  }

  loop = 0;
  transition_interpolator_ratio.reserve(transition_interpolator.size());
  for (unsigned int i=0; i<transition_interpolator.size(); i++ ) transition_interpolator_ratio[i] = 0;

  return RTC::RTC_OK;
}



RTC::ReturnCode_t ReferenceForceUpdater::onFinalize()
{
  std::cerr << "[" << m_profile.instance_name << "] onFinalize()" << std::endl;
  for ( std::map<std::string, interpolator*>::iterator it = ref_force_interpolator.begin(); it != ref_force_interpolator.end(); it++ ) {
    delete it->second;
  }
  for ( std::map<std::string, interpolator*>::iterator it = transition_interpolator.begin(); it != transition_interpolator.end(); it++ ) {
    delete it->second;
  }
  ref_force_interpolator.clear();
  transition_interpolator.clear();
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t ReferenceForceUpdater::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t ReferenceForceUpdater::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

RTC::ReturnCode_t ReferenceForceUpdater::onActivated(RTC::UniqueId ec_id)
{
  std::cerr << "[" << m_profile.instance_name<< "] onActivated(" << ec_id << ")" << std::endl;
  return RTC::RTC_OK;
}

RTC::ReturnCode_t ReferenceForceUpdater::onDeactivated(RTC::UniqueId ec_id)
{
  std::cerr << "[" << m_profile.instance_name<< "] onDeactivated(" << ec_id << ")" << std::endl;
  return RTC::RTC_OK;
}


#define DEBUGP ((m_debugLevel==1 && loop%200==0) || m_debugLevel > 1 )
RTC::ReturnCode_t ReferenceForceUpdater::onExecute(RTC::UniqueId ec_id)
{
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
  if (m_qRefIn.isNew()) {
    m_qRefIn.read();
  }

  //syncronize m_robot to the real robot
  if ( m_qRef.data.length() ==  m_robot->numJoints() ) {
    Guard guard(m_mutex);

    // Interpolator
    for (std::map<std::string, ReferenceForceUpdaterParam>::iterator itr = m_RFUParam.begin(); itr != m_RFUParam.end(); itr++ ) {
      std::string arm = itr->first;
      size_t arm_idx = ee_index_map[arm];
      bool transition_interpolator_isEmpty = transition_interpolator[arm]->isEmpty();
      if ( ! transition_interpolator_isEmpty ) {
        transition_interpolator[arm]->get(&transition_interpolator_ratio[arm_idx], true);
        if ( transition_interpolator[arm]->isEmpty() && m_RFUParam[arm].is_active && m_RFUParam[arm].is_stopping ) {
          std::cerr << "[" << m_profile.instance_name << "] ReferenceForceUpdater active => inactive." << std::endl;
          m_RFUParam[arm].is_active = false;
          m_RFUParam[arm].is_stopping = false;
        }
      }
    }
    // If RFU is not active
    {
      bool all_arm_is_not_active = true;
      for (std::map<std::string, ReferenceForceUpdaterParam>::iterator itr = m_RFUParam.begin(); itr != m_RFUParam.end(); itr++ ) {
        std::string arm = itr->first;
        size_t arm_idx = ee_index_map[arm];
        if ( m_RFUParam[arm].is_active ) all_arm_is_not_active = false;
        else for (unsigned int j=0; j<3; j++ ) ref_force[arm_idx](j) = m_ref_force_in[arm_idx].data[j];
      }
      //determin ref_force_out from ref_force_in
      if ( all_arm_is_not_active ) {
        for (unsigned int i=0; i<m_ref_force_in.size(); i++ ){
          for (unsigned int j=0; j<6; j++ ) {
            m_ref_force_out[i].data[j] = m_ref_force_in[i].data[j];
          }
          m_ref_force_out[i].tm = m_ref_force_in[i].tm;
          m_ref_forceOut[i]->write();
        }
        return RTC::RTC_OK;
      }
    }

    // If RFU is active
    {
      hrp::dvector qorg(m_robot->numJoints());

      // reference model
      for ( unsigned int i = 0; i < m_robot->numJoints(); i++ ){
        qorg[i] = m_robot->joint(i)->q;
        m_robot->joint(i)->q = m_qRef.data[i];
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
    }

    for (std::map<std::string, ReferenceForceUpdaterParam>::iterator itr = m_RFUParam.begin(); itr != m_RFUParam.end(); itr++ ) {
      // Update reference force
      hrp::Vector3 internal_force = hrp::Vector3::Zero();
      std::string arm = itr->first;
      size_t arm_idx = ee_index_map[arm];
      double interpolation_time = 0;
      bool is_active = m_RFUParam[arm].is_active;
      if ( is_active && loop % m_RFUParam[arm].update_count == 0 ) {
        hrp::Link* target_link = m_robot->link(ee_map[arm].target_name);
        hrp::Vector3 abs_motion_dir, tmp_act_force, df;
        hrp::Matrix33 ee_rot, sensor_rot;
        ee_rot = target_link->R * ee_map[arm].localR;
        if ( m_RFUParam[arm].frame=="local" )
            abs_motion_dir = ee_rot * m_RFUParam[arm].motion_dir;
        else {
            hrp::Matrix33 current_foot_mid_rot;
            std::vector<hrp::Matrix33> foot_rot;
            std::vector<std::string> leg_names;
            leg_names.push_back("rleg");
            leg_names.push_back("lleg");
            for (size_t i = 0; i < leg_names.size(); i++) {
                hrp::Link* target_link = m_robot->link(ee_map[leg_names[i]].target_name);
                foot_rot.push_back(target_link->R * ee_map[leg_names[i]].localR);
            }
            rats::mid_rot(current_foot_mid_rot, 0.5, foot_rot[0], foot_rot[1]);
            abs_motion_dir = current_foot_mid_rot * m_RFUParam[arm].motion_dir;
        }
        for (size_t i = 0; i < 3; i++ ) tmp_act_force(i) = m_force[arm_idx].data[i];
        hrp::Sensor* sensor = m_robot->sensor(hrp::Sensor::FORCE, arm_idx);
        sensor_rot = sensor->link->R * sensor->localR;
        tmp_act_force = sensor_rot * tmp_act_force;
        // Calc abs force diff
        df = tmp_act_force - ref_force[arm_idx];
        double inner_product = 0;
        if ( ! std::fabs((abs_motion_dir.norm() - 0.0)) < 1e-5 ) {
          abs_motion_dir.normalize();
          inner_product = df.dot(abs_motion_dir);
          if ( ! (inner_product < 0 && ref_force[arm_idx].dot(abs_motion_dir) < 0.0) ) {
            hrp::Vector3 in_f = ee_rot * internal_force;
            ref_force[arm_idx] = ref_force[arm_idx].dot(abs_motion_dir) * abs_motion_dir + in_f + (m_RFUParam[arm].p_gain * inner_product * transition_interpolator_ratio[arm_idx]) * abs_motion_dir;
            interpolation_time = (1/m_RFUParam[arm].update_freq) * m_RFUParam[arm].update_time_ratio;
            if ( ref_force_interpolator[arm]->isEmpty() ) {
              ref_force_interpolator[arm]->setGoal(ref_force[arm_idx].data(), interpolation_time, true);
            }
          }
        }
        if ( DEBUGP ) {
          std::cerr << "[" << m_profile.instance_name << "] Updating reference force" << std::endl;
          std::cerr << "[" << m_profile.instance_name << "]   inner_product = " << inner_product << ", ref_force = " << ref_force[arm_idx].dot(abs_motion_dir) << ", interpolation_time = " << interpolation_time << "[s]" << std::endl;
          std::cerr << "[" << m_profile.instance_name << "]   new ref_force = " << ref_force[arm_idx].format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "    [", "]")) << std::endl;
          std::cerr << "[" << m_profile.instance_name << "]   act_force = " << tmp_act_force.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "    [", "]")) << std::endl;
          std::cerr << "[" << m_profile.instance_name << "]   df = " << df.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "    [", "]")) << std::endl;
        }
      }
      if (!ref_force_interpolator[arm]->isEmpty()) {
        ref_force_interpolator[arm]->get(ref_force[arm_idx].data(), true);
      }
    }
  }

  //determin ref_force_out from ref_force_in
  for (unsigned int i=0; i<m_ref_force_in.size(); i++ ){
    for (unsigned int j=0; j<6; j++ ) {
      m_ref_force_out[i].data[j] = m_ref_force_in[i].data[j];
    }
    for (unsigned int j=0; j<3; j++ ) {
      m_ref_force_out[i].data[j] = ref_force[i](j) * transition_interpolator_ratio[i] + m_ref_force_in[i].data[j] * (1-transition_interpolator_ratio[i]);
    }
    m_ref_force_out[i].tm = m_ref_force_in[i].tm;
    m_ref_forceOut[i]->write();
  }

  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t ReferenceForceUpdater::onAborting(RTC::UniqueId ec_id)
{
return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t ReferenceForceUpdater::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t ReferenceForceUpdater::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t ReferenceForceUpdater::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t ReferenceForceUpdater::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


bool ReferenceForceUpdater::setReferenceForceUpdaterParam(const std::string& i_name_, const OpenHRP::ReferenceForceUpdaterService::ReferenceForceUpdaterParam& i_param)
{
  std::string arm = std::string(i_name_);
  std::cerr << "[" << m_profile.instance_name << "] setReferenceForceUpdaterParam [" << i_name_ << "]" << std::endl;
  Guard guard(m_mutex);
  if ( m_RFUParam.find(i_name_) == m_RFUParam.end() ) {
    std::cerr << "[" << m_profile.instance_name << "] Could not found reference force updater param [" << i_name_ << "]" << std::endl;
    return false;
  }
  if ( m_RFUParam[arm].is_active ) {
    std::cerr << "[" << m_profile.instance_name << "] Could not set parameters because rfu [" << i_name_ << "] is active" << std::endl;
    return false;
  }
  if ( std::string(i_param.frame) != "local" && std::string(i_param.frame) != "world" ) {
    std::cerr << "[" << m_profile.instance_name << "] \"frame\" parameter must be local/world. could not set \"" << std::string(i_param.frame) << "\"" <<std::endl;
    return false;
  }
  m_RFUParam[arm].p_gain = i_param.p_gain;
  m_RFUParam[arm].d_gain = i_param.d_gain;
  m_RFUParam[arm].i_gain = i_param.i_gain;
  m_RFUParam[arm].update_freq = i_param.update_freq;
  m_RFUParam[arm].update_time_ratio = i_param.update_time_ratio;
  m_RFUParam[arm].update_count=round((1/m_RFUParam[arm].update_freq)/m_dt);
  m_RFUParam[arm].frame=std::string(i_param.frame);

  for (size_t i = 0; i < 3; i++ ) m_RFUParam[arm].motion_dir(i) = i_param.motion_dir[i];

  std::cerr << "[" << m_profile.instance_name << "]   p_gain = " << m_RFUParam[arm].p_gain << ", d_gain = " << m_RFUParam[arm].d_gain << ", i_gain = " << m_RFUParam[arm].i_gain << std::endl;
  std::cerr << "[" << m_profile.instance_name << "]   update_freq = " << m_RFUParam[arm].update_freq << "[Hz], update_time_ratio = " << m_RFUParam[arm].update_time_ratio << std::endl;
  std::cerr << "[" << m_profile.instance_name << "]   motion_dir = " << m_RFUParam[arm].motion_dir.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "    [", "]")) << std::endl;
  std::cerr << "[" << m_profile.instance_name << "]   frame = " << m_RFUParam[arm].frame << std::endl;
  return true;
};

bool ReferenceForceUpdater::getReferenceForceUpdaterParam(const std::string& i_name_, OpenHRP::ReferenceForceUpdaterService::ReferenceForceUpdaterParam_out i_param)
{
  std::string arm = std::string(i_name_);
  std::cerr << "[" << m_profile.instance_name << "] getReferenceForceUpdaterParam [" << i_name_ << "]" << std::endl;
  if ( m_RFUParam.find(i_name_) == m_RFUParam.end() ) {
    std::cerr << "[" << m_profile.instance_name << "] Could not found reference force updater param [" << i_name_ << "]" << std::endl;
    return false;
  }
  Guard guard(m_mutex);
  i_param->p_gain = m_RFUParam[arm].p_gain;
  i_param->d_gain = m_RFUParam[arm].d_gain;
  i_param->i_gain = m_RFUParam[arm].i_gain;
  i_param->update_freq = m_RFUParam[arm].update_freq;
  i_param->update_time_ratio = m_RFUParam[arm].update_time_ratio;
  i_param->frame = m_RFUParam[arm].frame.c_str();
  for (size_t i = 0; i < 3; i++ ) i_param->motion_dir[i] = m_RFUParam[arm].motion_dir(i);
  return true;
};

bool ReferenceForceUpdater::startReferenceForceUpdater(const std::string& i_name_)
{
  std::cerr << "[" << m_profile.instance_name << "] startReferenceForceUpdater [" << i_name_ << "]" << std::endl;
  {
    Guard guard(m_mutex);
    if ( m_RFUParam.find(i_name_) == m_RFUParam.end() ) {
      std::cerr << "[" << m_profile.instance_name << "] Could not found reference force updater param [" << i_name_ << "]" << std::endl;
      return false;
    }
    if ( m_RFUParam[i_name_].is_active == true )
      return true;
    if (transition_interpolator[i_name_]->isEmpty()) {
      m_RFUParam[i_name_].is_active = true;
      double tmpstart = 0.0, tmpgoal = 1.0;
      size_t arm_idx = ee_index_map[i_name_];
      hrp::Vector3 currentRefForce( m_ref_force_in[arm_idx].data[0], m_ref_force_in[arm_idx].data[1], m_ref_force_in[arm_idx].data[2] );
      ref_force_interpolator[i_name_]->set(currentRefForce.data());
      transition_interpolator[i_name_]->set(&tmpstart);
      transition_interpolator[i_name_]->setGoal(&tmpgoal, 1.0, true);
    } else {
      return false;
    }
  }
  while (!transition_interpolator[i_name_]->isEmpty()) usleep(1000);
  usleep(1000);
  return true;
};

bool ReferenceForceUpdater::stopReferenceForceUpdater(const std::string& i_name_)
{
  std::cerr << "[" << m_profile.instance_name << "] stopReferenceForceUpdater [" << i_name_ << "]" << std::endl;
  {
    Guard guard(m_mutex);
    if ( m_RFUParam.find(i_name_) == m_RFUParam.end() ) {
      std::cerr << "[" << m_profile.instance_name << "] Could not found reference force updater param [" << i_name_ << "]" << std::endl;
      return false;
    }
    if ( m_RFUParam[i_name_].is_active == false ){//already not active
      return true;
    }
    double tmpstart = 1.0, tmpgoal = 0.0;
    transition_interpolator[i_name_]->set(&tmpstart);
    transition_interpolator[i_name_]->setGoal(&tmpgoal, 1.0, true);
    m_RFUParam[i_name_].is_stopping = true;
  }
  while (!transition_interpolator[i_name_]->isEmpty()) usleep(1000);
  usleep(1000);
  return true;
};

extern "C"
{

  void ReferenceForceUpdaterInit(RTC::Manager* manager)
  {
    RTC::Properties profile(ReferenceForceUpdater_spec);
    manager->registerFactory(profile,
                             RTC::Create<ReferenceForceUpdater>,
                             RTC::Delete<ReferenceForceUpdater>);
  }

};


