// -*- C++ -*-
/*!
 * @file  ObjectContactTurnaroundDetector.cpp
 * @brief object contact turnaround detector component
 * $Date$
 *
 * $Id$
 */

#include <rtm/CorbaNaming.h>
#include <hrpModel/Link.h>
#include <hrpModel/Sensor.h>
#include <hrpModel/ModelLoaderUtil.h>
#include "ObjectContactTurnaroundDetector.h"
#include "../ImpedanceController/RatsMatrix.h"
#include "hrpsys/util/Hrpsys.h"
#include <boost/assign.hpp>

typedef coil::Guard<coil::Mutex> Guard;

// Module specification
// <rtc-template block="module_spec">
static const char* objectcontactturnarounddetector_spec[] =
    {
        "implementation_id", "ObjectContactTurnaroundDetector",
        "type_name",         "ObjectContactTurnaroundDetector",
        "description",       "object contact turnaround detector component",
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

ObjectContactTurnaroundDetector::ObjectContactTurnaroundDetector(RTC::Manager* manager)
    : RTC::DataFlowComponentBase(manager),
      // <rtc-template block="initializer">
      m_qCurrentIn("qCurrent", m_qCurrent),
      m_rpyIn("rpy", m_rpy),
      m_contactStatesIn("contactStates", m_contactStates),
      m_otdDataOut("otdData", m_otdData),
      m_ObjectContactTurnaroundDetectorServicePort("ObjectContactTurnaroundDetectorService"),
      // </rtc-template>
      m_robot(hrp::BodyPtr()),
      m_debugLevel(0),
      dummy(0)
{
    m_service0.otd(this);
}

ObjectContactTurnaroundDetector::~ObjectContactTurnaroundDetector()
{
}


RTC::ReturnCode_t ObjectContactTurnaroundDetector::onInitialize()
{
    std::cerr << "[" << m_profile.instance_name << "] onInitialize()" << std::endl;
    bindParameter("debugLevel", m_debugLevel, "0");

    // Registration: InPort/OutPort/Service
    // <rtc-template block="registration">
    // Set InPort buffers
    addInPort("qCurrent", m_qCurrentIn);
    addInPort("rpy", m_rpyIn);
    addInPort("contactStates", m_contactStatesIn);

    // Set OutPort buffer
    addOutPort("otdData", m_otdDataOut);
  
    // Set service provider to Ports
    m_ObjectContactTurnaroundDetectorServicePort.registerProvider("service0", "ObjectContactTurnaroundDetectorService", m_service0);
  
    // Set service consumers to Ports
  
    // Set CORBA Service Ports
    addPort(m_ObjectContactTurnaroundDetectorServicePort);
  
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


    // Setting for wrench data ports (real)
    std::vector<std::string> fsensor_names;
    //   find names for real force sensors
    unsigned int npforce = m_robot->numSensors(hrp::Sensor::FORCE);
    for (unsigned int i=0; i<npforce; i++){
        fsensor_names.push_back(m_robot->sensor(hrp::Sensor::FORCE, i)->name);
    }
    //   add ports for all force sensors
    unsigned int nforce  = npforce;
    m_force.resize(nforce);
    m_forceIn.resize(nforce);
    std::cerr << "[" << m_profile.instance_name << "] force sensor ports" << std::endl;
    for (unsigned int i=0; i<nforce; i++){
        m_forceIn[i] = new InPort<TimedDoubleSeq>(fsensor_names[i].c_str(), m_force[i]);
        m_force[i].data.length(6);
        registerInPort(fsensor_names[i].c_str(), *m_forceIn[i]);
        std::cerr << "[" << m_profile.instance_name << "]   name = " << fsensor_names[i] << std::endl;
    }

    unsigned int dof = m_robot->numJoints();
    for ( unsigned int i = 0 ; i < dof; i++ ){
      if ( (int)i != m_robot->joint(i)->jointId ) {
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
            eet.index = i;
            ee_map.insert(std::pair<std::string, ee_trans>(ee_name , eet));
            base_name_map.insert(std::pair<std::string, std::string>(ee_name, ee_base));
            std::cerr << "[" << m_profile.instance_name << "] End Effector [" << ee_name << "]" << ee_target << " " << ee_base << std::endl;
            std::cerr << "[" << m_profile.instance_name << "]   target = " << ee_target << ", base = " << ee_base << std::endl;
            std::cerr << "[" << m_profile.instance_name << "]   localPos = " << eet.localPos.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "    [", "]")) << "[m]" << std::endl;
            std::cerr << "[" << m_profile.instance_name << "]   localR = " << eet.localR.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", "\n", "    [", "]")) << std::endl;
        }
        m_contactStates.data.length(num);
    }

    // initialize sensor_names
    std::cerr << "[" << m_profile.instance_name << "] Add sensor_names" << std::endl;
    for (unsigned int i=0; i<m_forceIn.size(); i++){
        std::string sensor_name = m_forceIn[i]->name();
        hrp::ForceSensor* sensor = m_robot->sensor<hrp::ForceSensor>(sensor_name);
        std::string sensor_link_name;
        if ( sensor ) {
            // real force sensor
            sensor_link_name = sensor->link->name;
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
            std::cerr << "[" << m_profile.instance_name << "]   No such ee setting for " << sensor_name << " and " << sensor_link_name << "!!. sensor_name for " << sensor_name << " cannot be added!!" << std::endl;
            continue;
        }
        // 4. Set impedance param
        ee_map[ee_name].sensor_name = sensor_name;
        std::cerr << "[" << m_profile.instance_name << "]   sensor = " << sensor_name << ", sensor-link = " << sensor_link_name << ", ee_name = " << ee_name << ", ee-link = " << ee_map[ee_name].target_name << std::endl;
    }

    otd = boost::shared_ptr<ObjectContactTurnaroundDetectorBase>(new ObjectContactTurnaroundDetectorBase(m_dt));
    otd->setPrintStr(std::string(m_profile.instance_name));

    // allocate memory for outPorts
    loop = 0;
    m_otdData.data.length(4); // mode, raw, filtered, dfiltered

    return RTC::RTC_OK;
}



RTC::ReturnCode_t ObjectContactTurnaroundDetector::onFinalize()
{
    return RTC::RTC_OK;
}

/*
  RTC::ReturnCode_t ObjectContactTurnaroundDetector::onStartup(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t ObjectContactTurnaroundDetector::onShutdown(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

RTC::ReturnCode_t ObjectContactTurnaroundDetector::onActivated(RTC::UniqueId ec_id)
{
    std::cerr << "[" << m_profile.instance_name<< "] onActivated(" << ec_id << ")" << std::endl;
    return RTC::RTC_OK;
}

RTC::ReturnCode_t ObjectContactTurnaroundDetector::onDeactivated(RTC::UniqueId ec_id)
{
  std::cerr << "[" << m_profile.instance_name<< "] onDeactivated(" << ec_id << ")" << std::endl;
  return RTC::RTC_OK;
}

#define DEBUGP ((m_debugLevel==1 && loop%200==0) || m_debugLevel > 1 )
RTC::ReturnCode_t ObjectContactTurnaroundDetector::onExecute(RTC::UniqueId ec_id)
{
    //std::cout << "ObjectContactTurnaroundDetector::onExecute(" << ec_id << ")" << std::endl;
    loop ++;

    // check dataport input
    for (unsigned int i=0; i<m_forceIn.size(); i++){
        if ( m_forceIn[i]->isNew() ) {
            m_forceIn[i]->read();
        }
    }
    if (m_rpyIn.isNew()) {
      m_rpyIn.read();
    }
    if (m_qCurrentIn.isNew()) {
      m_qCurrentIn.read();
      m_otdData.tm = m_qCurrent.tm;
    }
    if (m_contactStatesIn.isNew()) {
      m_contactStatesIn.read();
    }
    bool is_contact = false;
    for (size_t i = 0; i < m_contactStates.data.length(); i++) {
      if (m_contactStates.data[i]) is_contact = true;
    }
    if ( m_qCurrent.data.length() ==  m_robot->numJoints() &&
         is_contact && // one or more limbs are in contact
         ee_map.find("rleg") != ee_map.end() && ee_map.find("lleg") != ee_map.end() ) { // if legged robot
        Guard guard(m_mutex);
        calcObjectContactTurnaroundDetectorState();
        m_otdDataOut.write();
    }
    return RTC::RTC_OK;
}

/*
  RTC::ReturnCode_t ObjectContactTurnaroundDetector::onAborting(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t ObjectContactTurnaroundDetector::onError(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t ObjectContactTurnaroundDetector::onReset(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t ObjectContactTurnaroundDetector::onStateUpdate(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t ObjectContactTurnaroundDetector::onRateChanged(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

void ObjectContactTurnaroundDetector::calcFootMidCoords (hrp::Vector3& new_foot_mid_pos, hrp::Matrix33& new_foot_mid_rot)
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

void ObjectContactTurnaroundDetector::calcFootOriginCoords (hrp::Vector3& foot_origin_pos, hrp::Matrix33& foot_origin_rot)
{
  std::vector<rats::coordinates> leg_c_v;
  hrp::Vector3 ez = hrp::Vector3::UnitZ();
  hrp::Vector3 ex = hrp::Vector3::UnitX();
  std::vector<std::string> leg_names;
  for ( std::map<std::string, ee_trans>::iterator it = ee_map.begin(); it != ee_map.end(); it++ ) {
      // If rleg or lleg, and if reference contact states is true
      if (it->first.find("leg") != std::string::npos && m_contactStates.data[it->second.index]) leg_names.push_back(it->first);
  }
  for (size_t i = 0; i < leg_names.size(); i++) {
    hrp::Link* target_link = m_robot->link(ee_map[leg_names[i]].target_name);
    rats::coordinates leg_c(hrp::Vector3(target_link->p + target_link->R * ee_map[leg_names[i]].localPos), hrp::Matrix33(target_link->R * ee_map[leg_names[i]].localR));
    hrp::Vector3 xv1(leg_c.rot * ex);
    xv1(2)=0.0;
    xv1.normalize();
    hrp::Vector3 yv1(ez.cross(xv1));
    leg_c.rot(0,0) = xv1(0); leg_c.rot(1,0) = xv1(1); leg_c.rot(2,0) = xv1(2);
    leg_c.rot(0,1) = yv1(0); leg_c.rot(1,1) = yv1(1); leg_c.rot(2,1) = yv1(2);
    leg_c.rot(0,2) = ez(0); leg_c.rot(1,2) = ez(1); leg_c.rot(2,2) = ez(2);
    leg_c_v.push_back(leg_c);
  }
  if (leg_names.size() == 2) {
    rats::coordinates tmpc;
    rats::mid_coords(tmpc, 0.5, leg_c_v[0], leg_c_v[1]);
    foot_origin_pos = tmpc.pos;
    foot_origin_rot = tmpc.rot;
  } else { // size = 1
    foot_origin_pos = leg_c_v[0].pos;
    foot_origin_rot = leg_c_v[0].rot;
  }
}

void ObjectContactTurnaroundDetector::calcObjectContactTurnaroundDetectorState()
{
    // TODO
    // Currently only for legged robots
    // Set actual state
    for ( unsigned int i = 0; i < m_robot->numJoints(); i++ ) {
        m_robot->joint(i)->q = m_qCurrent.data[i];
    }
    updateRootLinkPosRot(m_rpy);
    m_robot->calcForwardKinematics();
    // Calc
    std::vector<hrp::Vector3> otd_forces, otd_moments, otd_hposv;
    hrp::Vector3 fmpos;
    hrp::Matrix33 fmrot, fmrotT;
    //calcFootMidCoords(fmpos, fmrot);
    calcFootOriginCoords(fmpos, fmrot);
    fmrotT = fmrot.transpose();
    for (unsigned int i=0; i<m_forceIn.size(); i++) {
        std::string sensor_name = m_forceIn[i]->name();
        if ( find(otd_sensor_names.begin(), otd_sensor_names.end(), sensor_name) != otd_sensor_names.end() ) {
            hrp::ForceSensor* sensor = m_robot->sensor<hrp::ForceSensor>(sensor_name);
            hrp::Vector3 data_p(m_force[i].data[0], m_force[i].data[1], m_force[i].data[2]);
            hrp::Vector3 data_r(m_force[i].data[3], m_force[i].data[4], m_force[i].data[5]);
            hrp::Matrix33 sensorR = sensor->link->R * sensor->localR;
            otd_forces.push_back(fmrotT*(sensorR*data_p));
            otd_moments.push_back(fmrotT*(sensorR*data_r));
            hrp::Vector3 eePos;
            for ( std::map<std::string, ee_trans>::iterator it = ee_map.begin(); it != ee_map.end(); it++ ) {
                if ( it->second.sensor_name == sensor_name ) {
                    ee_trans& eet = it->second;
                    hrp::Link* target_link = m_robot->link(eet.target_name);
                    eePos = fmrotT * (target_link->p + target_link->R * eet.localPos - fmpos);
                }
            }
            otd_hposv.push_back(eePos);
        }
    }
    otd->checkDetection(otd_forces, otd_moments, otd_hposv);
    // otdData
    m_otdData.data[0] = static_cast<double>(otd->getMode());
    m_otdData.data[1] = otd->getRawWrench();
    m_otdData.data[2] = otd->getFilteredWrench();
    m_otdData.data[3] = otd->getFilteredDwrench();
};

//

void ObjectContactTurnaroundDetector::updateRootLinkPosRot (TimedOrientation3D tmprpy)
{
  if ( m_robot->numSensors(hrp::Sensor::ACCELERATION) > 0) {
      hrp::Sensor *sensor = m_robot->sensor(hrp::Sensor::ACCELERATION, 0);
      hrp::Matrix33 tmpr;
      rats::rotm3times(tmpr, hrp::Matrix33(sensor->link->R*sensor->localR).transpose(), m_robot->rootLink()->R);
      rats::rotm3times(m_robot->rootLink()->R, hrp::rotFromRpy(tmprpy.data.r, tmprpy.data.p, tmprpy.data.y), tmpr);
      m_robot->rootLink()->p = hrp::Vector3::Zero();
  }
}

//
// ObjectContactTurnaroundDetector
//

void ObjectContactTurnaroundDetector::startObjectContactTurnaroundDetection(const double i_ref_diff_wrench, const double i_max_time, const OpenHRP::ObjectContactTurnaroundDetectorService::StrSequence& i_ee_names)
{
    Guard guard(m_mutex);
    otd->startDetection(i_ref_diff_wrench, i_max_time);
    otd_sensor_names.clear();
    for (size_t i = 0; i < i_ee_names.length(); i++) {
        otd_sensor_names.push_back(ee_map[std::string(i_ee_names[i])].sensor_name);
    }
}

OpenHRP::ObjectContactTurnaroundDetectorService::DetectorMode ObjectContactTurnaroundDetector::checkObjectContactTurnaroundDetection()
{
    OpenHRP::ObjectContactTurnaroundDetectorService::DetectorMode tmpmode;
    switch (otd->getMode()) {
    case ObjectContactTurnaroundDetectorBase::MODE_IDLE:
        tmpmode = ObjectContactTurnaroundDetectorService::MODE_DETECTOR_IDLE;
        break;
    case ObjectContactTurnaroundDetectorBase::MODE_STARTED:
        tmpmode = ObjectContactTurnaroundDetectorService::MODE_STARTED;
        break;
    case ObjectContactTurnaroundDetectorBase::MODE_DETECTED:
        tmpmode = ObjectContactTurnaroundDetectorService::MODE_DETECTED;
        break;
    case ObjectContactTurnaroundDetectorBase::MODE_MAX_TIME:
        tmpmode = ObjectContactTurnaroundDetectorService::MODE_MAX_TIME;
        break;
    default:
        tmpmode = ObjectContactTurnaroundDetectorService::MODE_DETECTOR_IDLE;
        break;
    }
    return tmpmode;
}

bool ObjectContactTurnaroundDetector::setObjectContactTurnaroundDetectorParam(const OpenHRP::ObjectContactTurnaroundDetectorService::objectContactTurnaroundDetectorParam &i_param_)
{
    Guard guard(m_mutex);
    std::cerr << "[" << m_profile.instance_name << "] setObjectContactTurnaroundDetectorParam" << std::endl;
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
    ObjectContactTurnaroundDetectorBase::detector_total_wrench dtw;
    switch (i_param_.detector_total_wrench) {
    case OpenHRP::ObjectContactTurnaroundDetectorService::TOTAL_FORCE:
        dtw = ObjectContactTurnaroundDetectorBase::TOTAL_FORCE;
        break;
    case OpenHRP::ObjectContactTurnaroundDetectorService::TOTAL_MOMENT:
        dtw = ObjectContactTurnaroundDetectorBase::TOTAL_MOMENT;
        break;
    case OpenHRP::ObjectContactTurnaroundDetectorService::TOTAL_MOMENT2:
        dtw = ObjectContactTurnaroundDetectorBase::TOTAL_MOMENT2;
        break;
    default:
        break;
    }
    otd->setDetectorTotalWrench(dtw);
    otd->printParams();
    return true;
};

bool ObjectContactTurnaroundDetector::getObjectContactTurnaroundDetectorParam(OpenHRP::ObjectContactTurnaroundDetectorService::objectContactTurnaroundDetectorParam& i_param_)
{
    std::cerr << "[" << m_profile.instance_name << "] getObjectContactTurnaroundDetectorParam" << std::endl;
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
    OpenHRP::ObjectContactTurnaroundDetectorService::DetectorTotalWrench dtw;
    switch (otd->getDetectorTotalWrench()) {
    case ObjectContactTurnaroundDetectorBase::TOTAL_FORCE:
        dtw = OpenHRP::ObjectContactTurnaroundDetectorService::TOTAL_FORCE;
        break;
    case ObjectContactTurnaroundDetectorBase::TOTAL_MOMENT:
        dtw = OpenHRP::ObjectContactTurnaroundDetectorService::TOTAL_MOMENT;
        break;
    case ObjectContactTurnaroundDetectorBase::TOTAL_MOMENT2:
        dtw = OpenHRP::ObjectContactTurnaroundDetectorService::TOTAL_MOMENT2;
        break;
    default:
        break;
    }
    i_param_.detector_total_wrench = dtw;
    return true;
}

bool ObjectContactTurnaroundDetector::getObjectForcesMoments(OpenHRP::ObjectContactTurnaroundDetectorService::Dbl3Sequence_out o_forces, OpenHRP::ObjectContactTurnaroundDetectorService::Dbl3Sequence_out o_moments, OpenHRP::ObjectContactTurnaroundDetectorService::DblSequence3_out o_3dofwrench, double& o_fric_coeff_wrench)
{
    std::cerr << "[" << m_profile.instance_name << "] getObjectForcesMoments" << std::endl;
    if (otd_sensor_names.size() == 0) return false;
    hrp::Vector3 tmpv = otd->getAxis() * otd->getFilteredWrench();
    o_forces = new OpenHRP::ObjectContactTurnaroundDetectorService::Dbl3Sequence ();
    o_moments = new OpenHRP::ObjectContactTurnaroundDetectorService::Dbl3Sequence ();
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
    o_3dofwrench = new OpenHRP::ObjectContactTurnaroundDetectorService::DblSequence3 ();
    o_3dofwrench->length(3);
    for (size_t i = 0; i < 3; i++) (*o_3dofwrench)[i] = tmpv(i);
    o_fric_coeff_wrench = otd->getFilteredFrictionCoeffWrench();
    return true;
}

extern "C"
{

    void ObjectContactTurnaroundDetectorInit(RTC::Manager* manager)
    {
        RTC::Properties profile(objectcontactturnarounddetector_spec);
        manager->registerFactory(profile,
                                 RTC::Create<ObjectContactTurnaroundDetector>,
                                 RTC::Delete<ObjectContactTurnaroundDetector>);
    }

};
