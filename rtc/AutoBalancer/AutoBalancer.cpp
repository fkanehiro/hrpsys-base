// -*- C++ -*-
/*!
 * @file  AutoBalancer.cpp
 * @brief autobalancer component
 * $Date$
 *
 * $Id$
 */

#include <rtm/CorbaNaming.h>
#include <hrpModel/Link.h>
#include <hrpModel/Sensor.h>
#include <hrpModel/ModelLoaderUtil.h>
#include "AutoBalancer.h"
#include <hrpModel/JointPath.h>
#include <hrpUtil/MatrixSolvers.h>
#include "util/Hrpsys.h"


#define MAX_TRANSITION_COUNT (2/m_dt)
typedef coil::Guard<coil::Mutex> Guard;
using namespace rats;

// Module specification
// <rtc-template block="module_spec">
static const char* autobalancer_spec[] =
    {
        "implementation_id", "AutoBalancer",
        "type_name",         "AutoBalancer",
        "description",       "autobalancer component",
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

AutoBalancer::AutoBalancer(RTC::Manager* manager)
    : RTC::DataFlowComponentBase(manager),
      // <rtc-template block="initializer">
      m_qRefIn("qRef", m_qRef),
      m_qCurrentIn("qCurrent", m_qCurrent),
      m_qOut("q", m_qRef),
      m_zmpRefOut("zmpRef", m_zmpRef),
      m_basePosOut("basePos", m_basePos),
      m_baseRpyOut("baseRpy", m_baseRpy),
      m_baseTformOut("baseTformOut", m_baseTform),
      m_accRefOut("accRef", m_accRef),
      m_contactStatesOut("contactStates", m_contactStates),
      m_AutoBalancerServicePort("AutoBalancerService"),
      // </rtc-template>
      move_base_gain(0.1),
      m_robot(hrp::BodyPtr()),
      m_debugLevel(0)
{
    m_service0.autobalancer(this);
}

AutoBalancer::~AutoBalancer()
{
}


RTC::ReturnCode_t AutoBalancer::onInitialize()
{
    std::cout << "AutoBalancer::onInitialize()" << std::endl;
    bindParameter("debugLevel", m_debugLevel, "0");

    // Registration: InPort/OutPort/Service
    // <rtc-template block="registration">
    // Set InPort buffers
    addInPort("qRef", m_qRefIn);
    addInPort("qCurrent", m_qCurrentIn);

    // Set OutPort buffer
    addOutPort("q", m_qOut);
    addOutPort("zmpRef", m_zmpRefOut);
    addOutPort("basePos", m_basePosOut);
    addOutPort("baseRpy", m_baseRpyOut);
    addOutPort("baseTformOut", m_baseTformOut);
    addOutPort("accRef", m_accRefOut);
    addOutPort("contactStates", m_contactStatesOut);
  
    // Set service provider to Ports
    m_AutoBalancerServicePort.registerProvider("service0", "AutoBalancerService", m_service0);
  
    // Set service consumers to Ports
  
    // Set CORBA Service Ports
    addPort(m_AutoBalancerServicePort);
  
    // </rtc-template>
    // <rtc-template block="bind_config">
    // Bind variables and configuration variable
  
    // </rtc-template>

    RTC::Properties& prop =  getProperties();
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
      std::cerr << "failed to load model[" << prop["model"] << "] in "
                << m_profile.instance_name << std::endl;
      return RTC::RTC_ERROR;
    }

    // allocate memory for outPorts
    m_qRef.data.length(m_robot->numJoints());
    m_qCurrent.data.length(m_robot->numJoints());
    qorg.resize(m_robot->numJoints());
    qrefv.resize(m_robot->numJoints());
    m_baseTform.data.length(12);

    transition_count = 0;
    control_mode = MODE_IDLE;
    loop = 0;

    zmp_interpolate_time = 1.0;
    zmp_interpolator = new interpolator(6, m_dt);

    // setting from conf file
    // GaitGenerator requires abc_leg_offset and abc_stride_parameter in robot conf file
    // setting leg_pos from conf file
    coil::vstring leg_offset_str = coil::split(prop["abc_leg_offset"], ",");
    std::vector<hrp::Vector3> leg_pos;
    if (leg_offset_str.size() > 0) {
      hrp::Vector3 leg_offset;
      for (size_t i = 0; i < 3; i++) coil::stringTo(leg_offset(i), leg_offset_str[i].c_str());
      std::cerr << "[AutoBalancer] abc_leg_offset : " << leg_offset(0) << " " << leg_offset(1) << " " << leg_offset(2) << std::endl;
      leg_pos.push_back(hrp::Vector3(-1*leg_offset));
      leg_pos.push_back(hrp::Vector3(leg_offset));
    }
    // setting stride limitations from conf file
    coil::vstring stride_param_str = coil::split(prop["abc_stride_parameter"], ",");
    hrp::Vector3 stride_param;
    if (stride_param_str.size() > 0) {
      for (size_t i = 0; i < 3; i++) coil::stringTo(stride_param(i), stride_param_str[i].c_str());
      std::cerr << "[AutoBalancer] abc_stride_parameter : " << stride_param(0) << " " << stride_param(1) << " " << stride_param(2) << std::endl;
    }
    if (default_zmp_offsets.size() == 0) {
      for (size_t i = 0; i < 2; i++) default_zmp_offsets.push_back(hrp::Vector3::Zero());
    }
    if (leg_offset_str.size() > 0 && stride_param_str.size() > 0) {
      gg = ggPtr(new rats::gait_generator(m_dt, leg_pos, stride_param(0)/*[m]*/, stride_param(1)/*[m]*/, stride_param(2)/*[deg]*/));
      gg->set_default_zmp_offsets(default_zmp_offsets);
    }
    gg_is_walking = gg_ending = gg_solved = false;
    fix_leg_coords = coordinates();

    // setting from conf file
    // rleg,TARGET_LINK,BASE_LINK
    coil::vstring end_effectors_str = coil::split(prop["end_effectors"], ",");
    size_t prop_num = 10;
    if (end_effectors_str.size() > 0) {
      size_t num = end_effectors_str.size()/prop_num;
      for (size_t i = 0; i < num; i++) {
        std::string ee_name, ee_target, ee_base;
        coil::stringTo(ee_name, end_effectors_str[i*prop_num].c_str());
        coil::stringTo(ee_target, end_effectors_str[i*prop_num+1].c_str());
        coil::stringTo(ee_base, end_effectors_str[i*prop_num+2].c_str());
        ABCIKparam tp;
        for (size_t j = 0; j < 3; j++) {
          coil::stringTo(tp.target2foot_offset_pos(j), end_effectors_str[i*prop_num+3+j].c_str());
        }
        double tmpv[4];
        for (int j = 0; j < 4; j++ ) {
          coil::stringTo(tmpv[j], end_effectors_str[i*prop_num+6+j].c_str());
        }
        tp.target2foot_offset_rot = Eigen::AngleAxis<double>(tmpv[3], hrp::Vector3(tmpv[0], tmpv[1], tmpv[2])).toRotationMatrix(); // rotation in VRML is represented by axis + angle
        tp.base_name = ee_base;
        tp.target_name = ee_target;
        tp.manip = hrp::JointPathExPtr(new hrp::JointPathEx(m_robot, m_robot->link(tp.base_name),
                                                            m_robot->link(tp.target_name)));
        ikp.insert(std::pair<std::string, ABCIKparam>(ee_name , tp));
        std::cerr << "abc limb[" << ee_name << "] " << ee_target << " " << ee_base << std::endl;
        std::cerr << "     offset_pos : " << tp.target2foot_offset_pos(0) << " " << tp.target2foot_offset_pos(1) << " " << tp.target2foot_offset_pos(2) << std::endl;
        contact_states_index_map.insert(std::pair<std::string, size_t>(ee_name, i));
      }
      m_contactStates.data.length(num);
    }

    // ref force port
    coil::vstring virtual_force_sensor = coil::split(prop["virtual_force_sensor"], ",");
    int npforce = m_robot->numSensors(hrp::Sensor::FORCE);
    int nvforce = virtual_force_sensor.size()/10;
    int nforce  = npforce + nvforce;
    m_ref_force.resize(nforce);
    m_ref_forceIn.resize(nforce);
    for (unsigned int i=0; i<npforce; i++){
        hrp::Sensor *s = m_robot->sensor(hrp::Sensor::FORCE, i);
        m_ref_forceIn[i] = new InPort<TimedDoubleSeq>(std::string("ref_"+s->name).c_str(), m_ref_force[i]);
        m_ref_force[i].data.length(6);
        registerInPort(std::string("ref_"+s->name).c_str(), *m_ref_forceIn[i]);
        std::cerr << s->name << std::endl;
    }
    for (unsigned int i=0; i<nvforce; i++){
        std::string name = virtual_force_sensor[i*10+0];
        m_ref_forceIn[i+npforce] = new InPort<TimedDoubleSeq>(std::string("ref_"+name).c_str(), m_ref_force[i+npforce]);
        m_ref_force[i+npforce].data.length(6);
        registerInPort(std::string("ref_"+name).c_str(), *m_ref_forceIn[i+npforce]);
    }
    for (unsigned int i=0; i<m_ref_forceIn.size(); i++){
      ref_forces.push_back(hrp::Vector3(0,0,0));
    }
    sbp_offset = hrp::Vector3(0,0,0);
    sbp_cog_offset = hrp::Vector3(0,0,0);
    //use_force = MODE_NO_FORCE;
    use_force = MODE_REF_FORCE;

    if (ikp.find("rleg") != ikp.end() && ikp.find("lleg") != ikp.end()) {
      is_legged_robot = true;
    } else {
      is_legged_robot = false;
    }

    m_accRef.data.ax = m_accRef.data.ay = m_accRef.data.az = 0.0;
    prev_imu_sensor_vel = hrp::Vector3::Zero();

    return RTC::RTC_OK;
}



RTC::ReturnCode_t AutoBalancer::onFinalize()
{
  delete zmp_interpolator;
  return RTC::RTC_OK;
}

/*
  RTC::ReturnCode_t AutoBalancer::onStartup(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t AutoBalancer::onShutdown(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

RTC::ReturnCode_t AutoBalancer::onActivated(RTC::UniqueId ec_id)
{
    std::cout << "AutoBalancer::onActivated(" << ec_id << ")" << std::endl;
    
    return RTC::RTC_OK;
}

/*
  RTC::ReturnCode_t AutoBalancer::onDeactivated(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

#define DEBUGP ((m_debugLevel==1 && loop%200==0) || m_debugLevel > 1 )
//#define DEBUGP2 ((loop%200==0))
#define DEBUGP2 (false)
RTC::ReturnCode_t AutoBalancer::onExecute(RTC::UniqueId ec_id)
{
  // std::cerr << "AutoBalancer::onExecute(" << ec_id << ")" << std::endl;
    loop ++;
    if (m_qRefIn.isNew()) {
        m_qRefIn.read();
    }
    if (m_qCurrentIn.isNew()) {
        m_qCurrentIn.read();
    }
    for (unsigned int i=0; i<m_ref_forceIn.size(); i++){
        if ( m_ref_forceIn[i]->isNew() ) {
            m_ref_forceIn[i]->read();
        }
    }
    Guard guard(m_mutex);
    if ( is_legged_robot ) {
      getCurrentParameters();
      getTargetParameters();
      if (control_mode == MODE_ABC ) {
        solveLimbIK();
      } else {
        for ( std::map<std::string, ABCIKparam>::iterator it = ikp.begin(); it != ikp.end(); it++ ) {
          if (it->first == "rleg" || it->first == "lleg") {
            it->second.current_p0 = m_robot->link(it->second.target_name)->p;
            it->second.current_r0 = m_robot->link(it->second.target_name)->R;
          }
        }
      }
    }
    if ( m_qRef.data.length() != 0 ) { // initialized
      if (is_legged_robot) {
        for ( int i = 0; i < m_robot->numJoints(); i++ ){
          m_qRef.data[i] = m_robot->joint(i)->q;
        }
      }
      m_qOut.write();
    }
    hrp::Vector3 baseRpy = hrp::rpyFromRot(m_robot->rootLink()->R);
    m_baseRpy.data.r = baseRpy(0);
    m_baseRpy.data.p = baseRpy(1);
    m_baseRpy.data.y = baseRpy(2);
    m_baseRpy.tm = m_qRef.tm;
    m_baseRpyOut.write();
    m_basePos.data.x = m_robot->rootLink()->p(0);
    m_basePos.data.y = m_robot->rootLink()->p(1);
    m_basePos.data.z = m_robot->rootLink()->p(2);
    m_basePos.tm = m_qRef.tm;
    m_basePosOut.write();
    double *tform_arr = m_baseTform.data.get_buffer();
    tform_arr[0] = m_basePos.data.x;
    tform_arr[1] = m_basePos.data.y;
    tform_arr[2] = m_basePos.data.z;
    hrp::Matrix33 Rot = hrp::rotFromRpy(m_baseRpy.data.r, 
                                        m_baseRpy.data.p, 
                                        m_baseRpy.data.y); 
    hrp::setMatrix33ToRowMajorArray(Rot, tform_arr, 3);
    m_baseTform.tm = m_qRef.tm;
    m_baseTformOut.write();

    m_zmpRef.data.x = ref_zmp(0);
    m_zmpRef.data.y = ref_zmp(1);
    m_zmpRef.data.z = ref_zmp(2);
    m_zmpRef.tm = m_qRef.tm;
    m_zmpRefOut.write();

    hrp::Sensor* sen = m_robot->sensor<hrp::RateGyroSensor>("gyrometer");
    if (sen != NULL) {
      hrp::Vector3 imu_sensor_pos = sen->link->p + sen->link->R * sen->localPos;
      hrp::Vector3 imu_sensor_vel = (imu_sensor_pos - prev_imu_sensor_pos)/m_dt;
      // convert to imu sensor local acceleration
      hrp::Vector3 acc = (sen->link->R * sen->localR).transpose() * (imu_sensor_vel - prev_imu_sensor_vel)/m_dt;
      m_accRef.data.ax = acc(0); m_accRef.data.ay = acc(1); m_accRef.data.az = acc(2);
      m_accRefOut.write();
      prev_imu_sensor_pos = imu_sensor_pos;
      prev_imu_sensor_vel = imu_sensor_vel;
    }

    m_contactStatesOut.write();

    return RTC::RTC_OK;
}

void AutoBalancer::getCurrentParameters()
{
  current_root_p = m_robot->rootLink()->p;
  current_root_R = m_robot->rootLink()->R;
  for ( int i = 0; i < m_robot->numJoints(); i++ ){
    qorg[i] = m_robot->joint(i)->q;
  }
}

void AutoBalancer::getTargetParameters()
{
  if ( transition_count == 0 ) {
    transition_smooth_gain = 1.0;
  } else {
    transition_smooth_gain = 1/(1+exp(-9.19*(((MAX_TRANSITION_COUNT - std::fabs(transition_count)) / MAX_TRANSITION_COUNT) - 0.5)));
  }
  if ( transition_count > 0 ) {
    for ( int i = 0; i < m_robot->numJoints(); i++ ){
      m_robot->joint(i)->q = ( m_qRef.data[i] - transition_joint_q[i] ) * transition_smooth_gain + transition_joint_q[i];
    }
  } else {
    for ( int i = 0; i < m_robot->numJoints(); i++ ){
      m_robot->joint(i)->q = m_qRef.data[i];
    }
  }
  for ( int i = 0; i < m_robot->numJoints(); i++ ){
    qrefv[i] = m_robot->joint(i)->q;
  }
  if ( transition_count < 0 ) {
    transition_count++;
  } else if ( transition_count > 0 ) {
    transition_count--;
    if(transition_count <= 0){ // erase impedance param
      std::cerr << "Finished cleanup" << std::endl;
      control_mode = MODE_IDLE;
    }
  }
  m_robot->calcForwardKinematics();
  coordinates rc, lc;
  if ( is_legged_robot ) {
    coordinates tmp_fix_coords;
    if (!zmp_interpolator->isEmpty()) {
      double default_zmp_offsets_output[6];
      zmp_interpolator->get(default_zmp_offsets_output, true);
      for (size_t i = 0; i < 2; i++)
        for (size_t j = 0; j < 3; j++)
          default_zmp_offsets[i](j) = default_zmp_offsets_output[i*3+j];
      if (DEBUGP) {
        std::cerr << "default_zmp_offsets (interpolate): "
                  << default_zmp_offsets[0](0) << " " << default_zmp_offsets[0](1) << " " << default_zmp_offsets[0](2) << " "
                  << default_zmp_offsets[1](0) << " " << default_zmp_offsets[1](1) << " " << default_zmp_offsets[1](2) << std::endl;
      }
    }
    if ( gg_is_walking ) {
      gg->set_default_zmp_offsets(default_zmp_offsets);
      gg_solved = gg->proc_one_tick();
      coordinates sp_coords(gg->get_support_leg_coords().pos, gg->get_support_leg_coords().rot);
      coordinates sw_coords(gg->get_swing_leg_coords().pos, gg->get_swing_leg_coords().rot);
      coordinates tmpc;
      coordinates(ikp[gg->get_support_leg()].target2foot_offset_pos, ikp[gg->get_support_leg()].target2foot_offset_rot).inverse_transformation(tmpc);
      sp_coords.transform(tmpc);
      ikp[gg->get_support_leg()].target_p0 = sp_coords.pos;
      ikp[gg->get_support_leg()].target_r0 = sp_coords.rot;
      coordinates(ikp[gg->get_swing_leg()].target2foot_offset_pos, ikp[gg->get_swing_leg()].target2foot_offset_rot).inverse_transformation(tmpc);
      sw_coords.transform(tmpc);
      ikp[gg->get_swing_leg()].target_p0 = sw_coords.pos;
      ikp[gg->get_swing_leg()].target_r0 = sw_coords.rot;
      gg->get_swing_support_mid_coords(tmp_fix_coords);
      // TODO : assume biped
      switch (gg->get_current_support_state()) {
      case 0:
        m_contactStates.data[contact_states_index_map["rleg"]] = true;
        m_contactStates.data[contact_states_index_map["lleg"]] = true;
        break;
      case 1:
        m_contactStates.data[contact_states_index_map["rleg"]] = true;
        m_contactStates.data[contact_states_index_map["lleg"]] = false;
        break;
      case 2:
        m_contactStates.data[contact_states_index_map["rleg"]] = false;
        m_contactStates.data[contact_states_index_map["lleg"]] = true;
        break;
      default:
        break;
      }
    } else {
      tmp_fix_coords = fix_leg_coords;
    }
    fixLegToCoords(":both", tmp_fix_coords);

    /* update ref_forces ;; sp's absolute -> rmc's absolute */
    for (size_t i = 0; i < m_ref_forceIn.size(); i++)
      tmp_fix_coords.rotate_vector(ref_forces[i],
                                   hrp::Vector3(m_ref_force[i].data[0], m_ref_force[i].data[1], m_ref_force[i].data[2]));
    tmp_fix_coords.rotate_vector(sbp_offset, hrp::Vector3(sbp_offset));

    target_root_p = m_robot->rootLink()->p;
    target_root_R = m_robot->rootLink()->R;
    for ( std::map<std::string, ABCIKparam>::iterator it = ikp.begin(); it != ikp.end(); it++ ) {
      if ( control_mode != MODE_ABC || it->first.find("leg") == std::string::npos ) {
        it->second.target_p0 = m_robot->link(it->second.target_name)->p;
        it->second.target_r0 = m_robot->link(it->second.target_name)->R;
      }
    }
    ikp["rleg"].getRobotEndCoords(rc, m_robot);
    ikp["lleg"].getRobotEndCoords(lc, m_robot);
    rc.translate(default_zmp_offsets[0]); /* rleg */
    lc.translate(default_zmp_offsets[1]); /* lleg */
    if (gg_is_walking) {
      ref_cog = gg->get_cog();
    } else {
      ref_cog = (rc.pos+lc.pos)/2.0;
    }
  }
  if (control_mode == MODE_IDLE) {
    if ( is_legged_robot ) {
      ref_zmp(0) = ref_cog(0);
      ref_zmp(1) = ref_cog(1);
      ref_zmp(2) = (rc.pos(2) + lc.pos(2)) / 2.0;
    } else ref_zmp = hrp::Vector3(0,0,0);
  } else if (gg_is_walking) {
    ref_zmp = gg->get_refzmp();
  } else {
    if ( is_legged_robot ) {
      ref_zmp(0) = ref_cog(0);
      ref_zmp(1) = ref_cog(1);
      ref_zmp(2) = (rc.pos(2) + lc.pos(2)) / 2.0;
    } else ref_zmp = hrp::Vector3(0,0,0);
  }
  if ( transition_count > 0 ) {
    ref_zmp = transition_smooth_gain * ( ref_zmp - prev_ref_zmp ) + prev_ref_zmp;
  }
}

void AutoBalancer::fixLegToCoords (const std::string& leg, const coordinates& coords)
{
  coordinates tar, ref, delta, tmp;
  coordinates rleg_endcoords, lleg_endcoords;
  ikp["rleg"].getRobotEndCoords(rleg_endcoords, m_robot);
  ikp["lleg"].getRobotEndCoords(lleg_endcoords, m_robot);
  mid_coords(tar, 0.5, rleg_endcoords , lleg_endcoords);
  tmp = coords;
  ref = coordinates(m_robot->rootLink()->p, m_robot->rootLink()->R);
  tar.transformation(delta, ref, ":local");
  tmp.transform(delta, ":local");
  m_robot->rootLink()->p = tmp.pos;
  m_robot->rootLink()->R = tmp.rot;
  m_robot->calcForwardKinematics();
}

bool AutoBalancer::solveLimbIKforLimb (ABCIKparam& param)
{
  param.current_p0 = m_robot->link(param.target_name)->p;
  param.current_r0 = m_robot->link(param.target_name)->R;

  hrp::Vector3 vel_p, vel_r;
  vel_p = param.target_p0 - param.current_p0;
  rats::difference_rotation(vel_r, param.current_r0, param.target_r0);
  param.manip->calcInverseKinematics2Loop(vel_p, vel_r, 1.0, 0.001, 0.01, &qrefv);
  return true;
}

void AutoBalancer::solveLimbIK ()
{
  for ( std::map<std::string, ABCIKparam>::iterator it = ikp.begin(); it != ikp.end(); it++ ) {
    if (it->second.is_active) {
      for ( int j = 0; j < it->second.manip->numJoints(); j++ ){
	int i = it->second.manip->joint(j)->jointId;
	m_robot->joint(i)->q = qorg[i];
      }
    }
  }
  m_robot->rootLink()->p = current_root_p;
  m_robot->rootLink()->R = current_root_R;
  m_robot->calcForwardKinematics();
  hrp::Vector3 tmp_input_sbp = hrp::Vector3(0,0,0);
  static_balance_point_proc_one(tmp_input_sbp, ref_zmp(2));
  hrp::Vector3 dif_cog = tmp_input_sbp - ref_cog;
  dif_cog(2) = m_robot->rootLink()->p(2) - target_root_p(2);
  m_robot->rootLink()->p = m_robot->rootLink()->p + -1 * move_base_gain * transition_smooth_gain * dif_cog;
  m_robot->rootLink()->R = target_root_R;
  m_robot->calcForwardKinematics();

  for ( std::map<std::string, ABCIKparam>::iterator it = ikp.begin(); it != ikp.end(); it++ ) {
    if (it->second.is_active) solveLimbIKforLimb(it->second);
  }
  if (gg_is_walking && !gg_solved) stopWalking ();
}

/*
  RTC::ReturnCode_t AutoBalancer::onAborting(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t AutoBalancer::onError(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t AutoBalancer::onReset(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t AutoBalancer::onStateUpdate(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t AutoBalancer::onRateChanged(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

void AutoBalancer::startABCparam(const OpenHRP::AutoBalancerService::StrSequence& limbs)
{
  std::cerr << "[AutoBalancer] start auto balancer mode" << std::endl;
  transition_count = -MAX_TRANSITION_COUNT; // when start impedance, count up to 0
  Guard guard(m_mutex);
  for ( std::map<std::string, ABCIKparam>::iterator it = ikp.begin(); it != ikp.end(); it++ ) {
    it->second.is_active = false;
  }

  for (size_t i = 0; i < limbs.length(); i++) {
    ABCIKparam& tmp = ikp[std::string(limbs[i])];
    tmp.is_active = true;
    std::cerr << "abc limb [" << std::string(limbs[i]) << "]" << std::endl;
  }

  for ( int i = 0; i < m_robot->numJoints(); i++ ){
    m_robot->joint(i)->q = m_qRef.data[i];
  }
  m_robot->calcForwardKinematics();
  fixLegToCoords(":both", fix_leg_coords);
  current_root_p = m_robot->rootLink()->p;
  current_root_R = m_robot->rootLink()->R;
  control_mode = MODE_ABC;
}

void AutoBalancer::stopABCparam()
{
  std::cerr << "[AutoBalancer] stop auto balancer mode" << std::endl;
  //Guard guard(m_mutex);
  transition_count = MAX_TRANSITION_COUNT; // when start impedance, count up to 0
  transition_joint_q.resize(m_robot->numJoints());
  for (int i = 0; i < m_robot->numJoints(); i++ ) {
    transition_joint_q[i] = m_robot->joint(i)->q;
  }
  prev_ref_zmp = ref_zmp;
  control_mode = MODE_SYNC;
  gg_ending = gg_solved = gg_is_walking = false;
}

void AutoBalancer::startWalking ()
{
  if ( control_mode != MODE_ABC ) {
    return_control_mode = control_mode;
    OpenHRP::AutoBalancerService::StrSequence fix_limbs;
    fix_limbs.length(2);
    fix_limbs[0] = "rleg";
    fix_limbs[1] = "lleg";
    startABCparam(fix_limbs);
    waitABCTransition();
  }
  hrp::Vector3 cog(m_robot->calcCM());
  std::string init_support_leg (gg->get_footstep_front_leg() == "rleg" ? "lleg" : "rleg");
  std::string init_swing_leg (gg->get_footstep_front_leg());
  coordinates spc, swc;
  gg->set_default_zmp_offsets(default_zmp_offsets);
  ikp[init_support_leg].getTargetEndCoords(spc);
  ikp[init_swing_leg].getTargetEndCoords(swc);
  gg->initialize_gait_parameter(cog, spc, swc);
  while ( !gg->proc_one_tick() );
  gg_is_walking = gg_solved = true;
  gg_ending = false;
}

void AutoBalancer::stopWalking ()
{
  if (!gg_ending){
    gg_ending = true; // tmpolary
    /* sync */
  } else {
    /* overwrite sequencer's angle-vector when finishing steps */
    coordinates rleg_endcoords, lleg_endcoords;
    ikp["rleg"].getTargetEndCoords(rleg_endcoords);
    ikp["lleg"].getTargetEndCoords(lleg_endcoords);
    mid_coords(fix_leg_coords, 0.5, rleg_endcoords, lleg_endcoords);
    fixLegToCoords(":both", fix_leg_coords);
    gg->clear_footstep_node_list();
    if (return_control_mode == MODE_IDLE) stopABCparam();
    gg_is_walking = false;
    gg_ending = false;
  }
}

bool AutoBalancer::startAutoBalancer (const OpenHRP::AutoBalancerService::StrSequence& limbs)
{
  if (control_mode == MODE_IDLE) {
    startABCparam(limbs);
    waitABCTransition();
    return_control_mode = MODE_ABC;
    return true;
  } else {
    return false;
  }
}

bool AutoBalancer::stopAutoBalancer ()
{
  if (control_mode == MODE_ABC) {
    stopABCparam();
    waitABCTransition();
    return true;
  } else {
    return false;
  }
}

void AutoBalancer::waitABCTransition()
{
  while (transition_count != 0) usleep(10);
  usleep(10);
}
bool AutoBalancer::goPos(const double& x, const double& y, const double& th)
{
  coordinates foot_midcoords;
  coordinates rleg_endcoords, lleg_endcoords;
  ikp["rleg"].getTargetEndCoords(rleg_endcoords);
  ikp["lleg"].getTargetEndCoords(lleg_endcoords);
  mid_coords(foot_midcoords, 0.5, rleg_endcoords, lleg_endcoords);
  gg->go_pos_param_2_footstep_list(x, y, th, foot_midcoords);
  gg->print_footstep_list();
  startWalking();
  return 0;
}

bool AutoBalancer::goVelocity(const double& vx, const double& vy, const double& vth)
{
  if (gg_is_walking && gg_solved) {
    gg->set_velocity_param(vx, vy, vth);
  } else {
    coordinates foot_midcoords;
    coordinates rleg_endcoords, lleg_endcoords;
    ikp["rleg"].getTargetEndCoords(rleg_endcoords);
    ikp["lleg"].getTargetEndCoords(lleg_endcoords);
    mid_coords(foot_midcoords, 0.5, rleg_endcoords, lleg_endcoords);
    gg->initialize_velocity_mode(foot_midcoords, vx, vy, vth);
    startWalking();
  }
  return 0;
}

bool AutoBalancer::goStop ()
{
  gg->finalize_velocity_mode();
  waitFootSteps();
  return true;
}

bool AutoBalancer::setFootSteps(const OpenHRP::AutoBalancerService::FootstepSequence& fs)
{
  std::cerr << "set_foot_steps" << std::endl;
  coordinates tmpfs, initial_support_coords, initial_input_coords, fstrans;
  ikp[std::string(fs[0].leg)].getCurrentEndCoords(initial_support_coords);
  memcpy(initial_input_coords.pos.data(), fs[0].pos, sizeof(double)*3);
  initial_input_coords.rot = (Eigen::Quaternion<double>(fs[0].rot[0], fs[0].rot[1], fs[0].rot[2], fs[0].rot[3])).normalized().toRotationMatrix(); // rtc: (x, y, z, w) but eigen: (w, x, y, z)

  gg->clear_footstep_node_list();
  for (size_t i = 0; i < fs.length(); i++) {
    std::string leg(fs[i].leg);
    if (leg == "rleg" || leg == "lleg") {
      memcpy(tmpfs.pos.data(), fs[i].pos, sizeof(double)*3);
      tmpfs.rot = (Eigen::Quaternion<double>(fs[i].rot[0], fs[i].rot[1], fs[i].rot[2], fs[i].rot[3])).normalized().toRotationMatrix(); // rtc: (x, y, z, w) but eigen: (w, x, y, z)
      initial_input_coords.transformation(fstrans, tmpfs);
      tmpfs = initial_support_coords;
      tmpfs.transform(fstrans);
      gg->append_footstep_node(leg, tmpfs);
    } else {
      std::cerr << "no such target : " << leg << std::endl;
      return false;
    }
  }
  std::cerr << "[AutoBalancer] : print footsteps " << std::endl;
  gg->append_finalize_footstep();
  gg->print_footstep_list();
  startWalking();
  return true;
}

void AutoBalancer::waitFootSteps()
{
  //while (gg_is_walking) usleep(10);
  while (gg_is_walking || transition_count != 0 )
    usleep(10);
  usleep(10);
  gg->set_offset_velocity_param(0,0,0);
}

bool AutoBalancer::setGaitGeneratorParam(const OpenHRP::AutoBalancerService::GaitGeneratorParam& i_param)
{
  gg->set_stride_parameters(i_param.stride_parameter[0], i_param.stride_parameter[1], i_param.stride_parameter[2]);
  gg->set_default_step_time(i_param.default_step_time);
  gg->set_default_step_height(i_param.default_step_height);
  gg->set_default_double_support_ratio(i_param.default_double_support_ratio);
  if (i_param.default_orbit_type == OpenHRP::AutoBalancerService::SHUFFLING) {
    gg->set_default_orbit_type(gait_generator::SHUFFLING);
  } else if (i_param.default_orbit_type == OpenHRP::AutoBalancerService::CYCLOID) {
    gg->set_default_orbit_type(gait_generator::CYCLOID);
  } else if (i_param.default_orbit_type == OpenHRP::AutoBalancerService::RECTANGLE) {
    gg->set_default_orbit_type(gait_generator::RECTANGLE);
  }
  return true;
};

bool AutoBalancer::getGaitGeneratorParam(OpenHRP::AutoBalancerService::GaitGeneratorParam& i_param)
{
  gg->get_stride_parameters(i_param.stride_parameter[0], i_param.stride_parameter[1], i_param.stride_parameter[2]);
  i_param.default_step_time = gg->get_default_step_time();
  i_param.default_step_height = gg->get_default_step_height();
  i_param.default_double_support_ratio = gg->get_default_double_support_ratio();
  if (gg->get_default_orbit_type() == gait_generator::SHUFFLING) {
    i_param.default_orbit_type = OpenHRP::AutoBalancerService::SHUFFLING;
  } else if (gg->get_default_orbit_type() == gait_generator::CYCLOID) {
    i_param.default_orbit_type = OpenHRP::AutoBalancerService::CYCLOID;
  } else if (gg->get_default_orbit_type() == gait_generator::RECTANGLE) {
    i_param.default_orbit_type = OpenHRP::AutoBalancerService::RECTANGLE;
  }
  return true;
};

bool AutoBalancer::setAutoBalancerParam(const OpenHRP::AutoBalancerService::AutoBalancerParam& i_param)
{
  double default_zmp_offsets_array[6];
  move_base_gain = i_param.move_base_gain;
  for (size_t i = 0; i < 2; i++)
    for (size_t j = 0; j < 3; j++)
      default_zmp_offsets_array[i*3+j] = i_param.default_zmp_offsets[i][j];
  zmp_interpolator->go(default_zmp_offsets_array, zmp_interpolate_time, true);
  std::cerr << "move_base_gain: " << move_base_gain << std::endl;
  std::cerr << "default_zmp_offsets: "
            << default_zmp_offsets_array[0] << " " << default_zmp_offsets_array[1] << " " << default_zmp_offsets_array[2] << " "
            << default_zmp_offsets_array[3] << " " << default_zmp_offsets_array[4] << " " << default_zmp_offsets_array[5] << std::endl;
  return true;
};

bool AutoBalancer::getAutoBalancerParam(OpenHRP::AutoBalancerService::AutoBalancerParam& i_param)
{
  i_param.move_base_gain = move_base_gain;
  for (size_t i = 0; i < 2; i++)
    for (size_t j = 0; j < 3; j++)
      i_param.default_zmp_offsets[i][j] = default_zmp_offsets[i](j);
  return true;
};

void AutoBalancer::copyRatscoords2Footstep(OpenHRP::AutoBalancerService::Footstep& out_fs, const rats::coordinates& in_fs)
{
  memcpy(out_fs.pos, in_fs.pos.data(), sizeof(double)*3);
  Eigen::Quaternion<double> qt(in_fs.rot);
  out_fs.rot[0] = qt.w();
  out_fs.rot[1] = qt.x();
  out_fs.rot[2] = qt.y();
  out_fs.rot[3] = qt.z();
};

bool AutoBalancer::getFootstepParam(OpenHRP::AutoBalancerService::FootstepParam& i_param)
{
  coordinates rleg_endcoords, lleg_endcoords;
  ikp["rleg"].getCurrentEndCoords(rleg_endcoords);
  ikp["lleg"].getCurrentEndCoords(lleg_endcoords);
  copyRatscoords2Footstep(i_param.rleg_coords, rleg_endcoords);
  copyRatscoords2Footstep(i_param.lleg_coords, lleg_endcoords);
  copyRatscoords2Footstep(i_param.support_leg_coords, gg->get_support_leg_coords());
  copyRatscoords2Footstep(i_param.swing_leg_coords, gg->get_swing_leg_coords());
  copyRatscoords2Footstep(i_param.swing_leg_src_coords, gg->get_swing_leg_src_coords());
  copyRatscoords2Footstep(i_param.swing_leg_dst_coords, gg->get_swing_leg_dst_coords());
  copyRatscoords2Footstep(i_param.dst_foot_midcoords, gg->get_dst_foot_midcoords());
  if (gg->get_support_leg() == "rleg") {
    i_param.support_leg = OpenHRP::AutoBalancerService::RLEG;
  } else {
    i_param.support_leg = OpenHRP::AutoBalancerService::LLEG;
  }
  switch ( gg->get_current_support_state() ) {
  case 0: i_param.support_leg_with_both = OpenHRP::AutoBalancerService::BOTH; break;
  case 1: i_param.support_leg_with_both = OpenHRP::AutoBalancerService::RLEG; break;
  case 2: i_param.support_leg_with_both = OpenHRP::AutoBalancerService::LLEG; break;
  default: break;
  }
  return true;
};

void AutoBalancer::static_balance_point_proc_one(hrp::Vector3& tmp_input_sbp, const double ref_com_height)
{
  hrp::Vector3 target_sbp = hrp::Vector3(0, 0, 0);
  hrp::Vector3 tmpcog = m_robot->calcCM();
  switch ( use_force ) {
  case MODE_REF_FORCE:
    calc_static_balance_point_from_forces(target_sbp, tmpcog, ref_com_height, ref_forces);
    tmp_input_sbp = target_sbp - sbp_offset;
    sbp_cog_offset = tmp_input_sbp - tmpcog;
    break;
  case MODE_NO_FORCE:
    tmp_input_sbp = tmpcog + sbp_cog_offset;
    break;
  default: break;
  }
};

void AutoBalancer::calc_static_balance_point_from_forces(hrp::Vector3& sb_point, const hrp::Vector3& tmpcog, const double ref_com_height, std::vector<hrp::Vector3>& tmp_forces)
{
  double gravity = 9.8; // [m/s^2]
  hrp::Vector3 denom, nume;
  /* sb_point[m] = nume[kg * m/s^2 * m] / denom[kg * m/s^2] */
  double mass = m_robot->totalMass();
  for (size_t j = 0; j < 2; j++) {
    nume(j) = mass * gravity * tmpcog(j);
    denom(j) = mass * gravity;
    for (size_t i = 0; i < m_ref_forceIn.size(); i++) {
      std::string sensor_name = m_ref_forceIn[i]->name();
      if ( sensor_name.find("hsensor") != std::string::npos ) { // tempolary to get arm force coords
        hrp::Sensor *sensor = m_robot->sensor(hrp::Sensor::FORCE, i);
        hrp::Vector3 fpos = sensor->link->p + (sensor->link->R) * sensor->localPos;
        nume(j) += ( (fpos(2) - ref_com_height) * tmp_forces[i](j) - fpos(j) * tmp_forces[i](2) );
        denom(j) -= tmp_forces[i](2);
      }
    }
    sb_point(j) = nume(j) / denom(j);
  }
  sb_point(2) = ref_com_height;
};

//
extern "C"
{

    void AutoBalancerInit(RTC::Manager* manager)
    {
        RTC::Properties profile(autobalancer_spec);
        manager->registerFactory(profile,
                                 RTC::Create<AutoBalancer>,
                                 RTC::Delete<AutoBalancer>);
    }

};


