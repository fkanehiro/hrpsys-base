// -*- C++ -*-
/*!
 * @file  KalmanFilter.cpp
 * @brief kalman filter
 * $Date$
 *
 * $Id$
 */

#include "KalmanFilter.h"
#include "util/VectorConvert.h"
#include <rtm/CorbaNaming.h>
#include <hrpModel/ModelLoaderUtil.h>
#include <math.h>
#include <hrpModel/Link.h>
#include <hrpModel/Sensor.h>

//#define USE_EKF

// Module specification
// <rtc-template block="module_spec">
static const char* kalmanfilter_spec[] =
  {
    "implementation_id", "KalmanFilter",
    "type_name",         "KalmanFilter",
    "description",       "kalman filter",
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

KalmanFilter::KalmanFilter(RTC::Manager* manager)
  : RTC::DataFlowComponentBase(manager),
    // <rtc-template block="initializer">
    m_rateIn("rate", m_rate),
    m_accIn("acc", m_acc),
    m_accRefIn("accRef", m_accRef),
    m_rpyIn("rpyIn", m_rate),
    m_qCurrentIn("qCurrent", m_qCurrent),
    m_rpyOut("rpy", m_rpy),
    m_rpyRawOut("rpy_raw", m_rpyRaw),
    m_baseRpyOut("base_rpy", m_baseRpy),
    m_KalmanFilterServicePort("KalmanFilterService"),
    // </rtc-template>
    m_robot(hrp::BodyPtr()),
    m_debugLevel(0),
    dummy(0),
    loop(0)
{
  m_service0.kalman(this);
}

KalmanFilter::~KalmanFilter()
{
}


#define DEBUGP ((m_debugLevel==1 && loop%200==0) || m_debugLevel > 1 )
RTC::ReturnCode_t KalmanFilter::onInitialize()
{
  std::cerr << "[" << m_profile.instance_name << "] onInitialize()" << std::endl;
  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  bindParameter("debugLevel", m_debugLevel, "0");
  
  // </rtc-template>

  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("rate", m_rateIn);
  addInPort("acc", m_accIn);
  addInPort("accRef", m_accRefIn);
  addInPort("rpyIn", m_rpyIn);
  addInPort("qCurrent", m_qCurrentIn);

  // Set OutPort buffer
  addOutPort("rpy", m_rpyOut);
  addOutPort("rpy_raw", m_rpyRawOut);
  addOutPort("base_rpy", m_baseRpyOut);

  // Set service provider to Ports
  m_KalmanFilterServicePort.registerProvider("service0", "KalmanFilterService", m_service0);
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  addPort(m_KalmanFilterServicePort);
  
  // </rtc-template>

  // Setup robot model
  RTC::Properties& prop = getProperties();
  if ( ! coil::stringTo(m_dt, prop["dt"].c_str()) ) {
    std::cerr << "[" << m_profile.instance_name << "]failed to get dt" << std::endl;
    return RTC::RTC_ERROR;
  }

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
    std::cerr << "[" << m_profile.instance_name << "]failed to load model[" << prop["model"] << "]" << std::endl;
  }

  m_rpy.data.r = 0;
  m_rpy.data.p = 0;
  m_rpy.data.y = 0;

  if (m_robot->numSensors(hrp::Sensor::ACCELERATION) > 0) {
    hrp::Sensor* sensor = m_robot->sensor(hrp::Sensor::ACCELERATION, 0);
    m_sensorR = sensor->link->R * sensor->localR;
  } else {
    m_sensorR = hrp::Matrix33::Identity();
  }
  rpy_kf.setParam(m_dt, 0.001, 0.003, 1000, std::string(m_profile.instance_name));
  rpy_kf.setSensorR(m_sensorR);
  rpy_kf.setRobot(m_robot);
  ekf_filter.setdt(m_dt);
  kf_algorithm = OpenHRP::KalmanFilterService::RPYKalmanFilter;
  m_qCurrent.data.length(m_robot->numJoints());

  return RTC::RTC_OK;
}



/*
  RTC::ReturnCode_t KalmanFilter::onFinalize()
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t KalmanFilter::onStartup(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t KalmanFilter::onShutdown(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

RTC::ReturnCode_t KalmanFilter::onActivated(RTC::UniqueId ec_id)
{
  std::cerr << "[" << m_profile.instance_name<< "] onActivated(" << ec_id << ")" << std::endl;
  return RTC::RTC_OK;
}

RTC::ReturnCode_t KalmanFilter::onDeactivated(RTC::UniqueId ec_id)
{
  std::cerr << "[" << m_profile.instance_name<< "] onDeactivated(" << ec_id << ")" << std::endl;
  return RTC::RTC_OK;
}

RTC::ReturnCode_t KalmanFilter::onExecute(RTC::UniqueId ec_id)
{
  loop++;
  static int initialize = 0;
  //std::cerr << m_profile.instance_name<< ": onExecute(" << ec_id << ") " << std::endl;
  if (m_rpyIn.isNew() ) {
    m_rpyIn.read();
    m_rpy.data.r = m_rate.data.avx;
    m_rpy.data.p = m_rate.data.avy;
    m_rpy.data.y = m_rate.data.avz;
    m_rpyOut.write();
    return RTC::RTC_OK;
  }
  if (m_rateIn.isNew()){
    m_rateIn.read();
  }
  if (m_qCurrentIn.isNew()) {
    m_qCurrentIn.read();
    for ( int i = 0; i < m_robot->numJoints(); i++ ){
      m_robot->joint(i)->q = m_qCurrent.data[i];
    }
  }
  double sx_ref = 0.0, sy_ref = 0.0, sz_ref = 0.0;
  if (m_accRefIn.isNew()){
    m_accRefIn.read();
    sx_ref = m_accRef.data.ax, sy_ref = m_accRef.data.ay, sz_ref = m_accRef.data.az;
  }
  if (m_accIn.isNew()){
    m_accIn.read();

    Eigen::Vector3d acc = m_sensorR * hrp::Vector3(m_acc.data.ax-sx_ref, m_acc.data.ay-sy_ref, m_acc.data.az-sz_ref); // transform to imaginary acc data
    Eigen::Vector3d gyro = m_sensorR * hrp::Vector3(m_rate.data.avx, m_rate.data.avy, m_rate.data.avz); // transform to imaginary rate data
    if (DEBUGP) {
        std::cerr << "[" << m_profile.instance_name << "] raw data acc : " << std::endl << acc << std::endl;
        std::cerr << "[" << m_profile.instance_name << "] raw data gyro : " << std::endl << gyro << std::endl;
    }
    hrp::Vector3 rpy, rpyRaw, baseRpy;
    if (kf_algorithm == OpenHRP::KalmanFilterService::QuaternionExtendedKalmanFilter) {
        ekf_filter.main_one(rpy, rpyRaw, acc, gyro);
    } else if (kf_algorithm == OpenHRP::KalmanFilterService::RPYKalmanFilter) {
        m_robot->calcForwardKinematics();
        hrp::Matrix33 BtoS, slR;
        if (m_robot->numSensors(hrp::Sensor::ACCELERATION) > 0) {
            hrp::Sensor* sensor = m_robot->sensor(hrp::Sensor::ACCELERATION, 0);
            slR = sensor->link->R;
            BtoS = (m_robot->rootLink()->R).transpose() * (slR * sensor->localR);
            if (DEBUGP) {
                std::cerr << "[sendor->link->R] : " << std::endl << sensor->link->R << std::endl;
                std::cerr << "[sensor->localR] : " << std::endl << sensor->localR << std::endl;
            }
        } else {
            BtoS = (m_robot->rootLink()->R).transpose();
            slR = hrp::Matrix33::Identity();
        }
        if (DEBUGP) {
            std::cerr << "[BtoS in rot] : " << std::endl << BtoS << std::endl;
            std::cerr << "[BtoS in rpy] : " << std::endl << hrp::rpyFromRot(BtoS) << std::endl;
        }
        rpy_kf.main_one(rpy, rpyRaw, baseRpy, acc, gyro, BtoS, slR);
    }
    m_rpyRaw.data.r = rpyRaw(0);
    m_rpyRaw.data.p = rpyRaw(1);
    m_rpyRaw.data.y = rpyRaw(2);
    m_rpy.data.r = rpy(0);
    m_rpy.data.p = rpy(1);
    m_rpy.data.y = rpy(2);
    m_baseRpy.data.r = baseRpy(0);
    m_baseRpy.data.p = baseRpy(1);
    m_baseRpy.data.y = baseRpy(2);


    m_rpyOut.write();
    m_rpyRawOut.write();
    m_baseRpyOut.write();
  }
  return RTC::RTC_OK;
}

/*
  RTC::ReturnCode_t KalmanFilter::onAborting(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t KalmanFilter::onError(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t KalmanFilter::onReset(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t KalmanFilter::onStateUpdate(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t KalmanFilter::onRateChanged(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

bool KalmanFilter::setKalmanFilterParam(const OpenHRP::KalmanFilterService::KalmanFilterParam& i_param)
{
    std::cerr << "[" << m_profile.instance_name << "] setKalmanFilterParam" << std::endl;
    rpy_kf.setParam(m_dt, i_param.Q_angle, i_param.Q_rate, i_param.R_angle, std::string(m_profile.instance_name));
    kf_algorithm = i_param.kf_algorithm;
    std::cerr << "[" << m_profile.instance_name << "]   kf_algorithm=" << (kf_algorithm==OpenHRP::KalmanFilterService::RPYKalmanFilter?"RPYKalmanFilter":"QuaternionExtendedKalmanFilter") << std::endl;
    return true;
}

bool KalmanFilter::resetKalmanFilterState()
{
    rpy_kf.resetKalmanFilterState();
};

bool KalmanFilter::getKalmanFilterParam(OpenHRP::KalmanFilterService::KalmanFilterParam& i_param)
{
  i_param.Q_angle = rpy_kf.getQangle();
  i_param.Q_rate = rpy_kf.getQrate();
  i_param.R_angle = rpy_kf.getRangle();
  i_param.kf_algorithm = kf_algorithm;
  return true;
}

extern "C"
{

  void KalmanFilterInit(RTC::Manager* manager)
  {
    RTC::Properties profile(kalmanfilter_spec);
    manager->registerFactory(profile,
                             RTC::Create<KalmanFilter>,
                             RTC::Delete<KalmanFilter>);
  }

};


