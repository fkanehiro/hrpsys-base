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
    m_rpyOut("rpy", m_rpy),
    m_rpyRawOut("rpy_raw", m_rpyRaw),
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
  std::cerr << m_profile.instance_name << ": onInitialize()" << std::endl;
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

  // Set OutPort buffer
  addOutPort("rpy", m_rpyOut);
  addOutPort("rpy_raw", m_rpyRawOut);

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

  Q_angle = 0.001;
  Q_rate = 0.003;
  R_angle = 0.03;
  r_filter.setF(1, -m_dt, 0, 1);
  r_filter.setP(0, 0, 0, 0);
  r_filter.setQ(Q_angle*m_dt, 0, 0, Q_rate*m_dt);
  r_filter.setR(R_angle);
  r_filter.setB(m_dt, 0);

  p_filter.setF(1, -m_dt, 0, 1);
  p_filter.setP(0, 0, 0, 0);
  p_filter.setQ(Q_angle*m_dt, 0, 0, Q_rate*m_dt);
  p_filter.setR(R_angle);
  p_filter.setB(m_dt, 0);

  y_filter.setF(1, -m_dt, 0, 1);
  y_filter.setP(0, 0, 0, 0);
  y_filter.setQ(Q_angle*m_dt, 0, 0, Q_rate*m_dt);
  y_filter.setR(R_angle);
  y_filter.setB(m_dt, 0);

  m_rpy.data.r = 0;
  m_rpy.data.p = 0;
  m_rpy.data.y = 0;

  if (m_robot->numSensors(hrp::Sensor::ACCELERATION) > 0) {
    hrp::Sensor* sensor = m_robot->sensor(hrp::Sensor::ACCELERATION, 0);
    m_sensorR = sensor->link->R * sensor->localR;
  } else {
    m_sensorR = hrp::Matrix33::Identity();
  }
  kf_algorithm = OpenHRP::KalmanFilterService::RPYKalmanFilter;

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
  std::cerr << m_profile.instance_name<< ": onActivated(" << ec_id << ")" << std::endl;
  return RTC::RTC_OK;
}

RTC::ReturnCode_t KalmanFilter::onDeactivated(RTC::UniqueId ec_id)
{
  std::cerr << m_profile.instance_name<< ": onDeactivated(" << ec_id << ")" << std::endl;
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
    if (kf_algorithm == OpenHRP::KalmanFilterService::QuaternionExtendedKalmanFilter) {
        ekf_filter.prediction(gyro, m_dt);
        ekf_filter.correction(acc, Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 0));
        /* ekf_filter.printAll(); */

        Eigen::Matrix<double, 7, 1> x = ekf_filter.getx();
        Eigen::Quaternion<double> q = Eigen::Quaternion<double>(x[0], x[1], x[2], x[3]);
        Eigen::Vector3d eulerZYX = q.toRotationMatrix().eulerAngles(2,1,0);
        m_rpy.data.y = eulerZYX(0);
        m_rpy.data.p = eulerZYX(1);
        m_rpy.data.r = eulerZYX(2);
    } else if (kf_algorithm == OpenHRP::KalmanFilterService::RPYKalmanFilter) {
        //
        // G = [ cosb, sinb sina, sinb cosa,
        //          0,      cosa,     -sina,
        //      -sinb, cosb sina, cosb cosa]
        // s = [sx, sy, sz]t ( accelerometer )
        // g = [ 0 0 -g]t
        // s = G g = [g sinb, -g cosb sina, -g cosb cosa]t
        // hrp::RateGyroSensor* sensor = m_robot->sensor<hrp::RateGyroSensor>("gyrometer");
        double g = sqrt(acc(0) * acc(0) + acc(1) * acc(1) + acc(2) * acc(2));
        // atan2(y, x) = atan(y/x)
        double a, b;
        b = atan2( - acc(0) / g, sqrt( acc(1)/g * acc(1)/g + acc(2)/g * acc(2)/g ) );
        a = atan2( ( acc(1)/g ), ( acc(2)/g ) );
        m_rpyRaw.data.r = a;
        m_rpyRaw.data.p = b;
        m_rpyRaw.data.y = 0;
        // #if 0
        //       // complementary filter
        //       m_rpy.data.r = 0.98 *(m_rpy.data.r+m_rate.data.avx*m_dt) + 0.02*m_rpyRaw.data.r;
        //       m_rpy.data.p = 0.98 *(m_rpy.data.p+m_rate.data.avy*m_dt) + 0.02*m_rpyRaw.data.p;
        //       m_rpy.data.y = 0.98 *(m_rpy.data.y+m_rate.data.avz*m_dt) + 0.02*m_rpyRaw.data.y;
        // #endif
        // kalman filter
        // x[0] = m_rpyRaw.data.r; // angle (from m/sec Acceleration3D, unstable, no drift )
        // x[1] = m_rate.data.avx; // rate ( rad/sec, AngularVelocity, gyro, stable/drift )
        // use kalman filter with imaginary data
        r_filter.update(gyro(0), m_rpyRaw.data.r);
        p_filter.update(gyro(1), m_rpyRaw.data.p);
        y_filter.update(gyro(2), m_rpyRaw.data.y);

        Eigen::AngleAxis<double> aaZ(y_filter.getx()[0], Eigen::Vector3d::UnitZ());
        Eigen::AngleAxis<double> aaY(p_filter.getx()[0], Eigen::Vector3d::UnitY());
        Eigen::AngleAxis<double> aaX(r_filter.getx()[0], Eigen::Vector3d::UnitX());
        Eigen::Quaternion<double> q = aaZ * aaY * aaX;
        hrp::Matrix33 imaginaryRotationMatrix = q.toRotationMatrix();
        hrp::Matrix33 realRotationMatrix = imaginaryRotationMatrix * m_sensorR; // inverse transform to real data
        hrp::Vector3 euler = realRotationMatrix.eulerAngles(2,1,0);
        m_rpy.data.y = euler(0);
        m_rpy.data.p = euler(1);
        m_rpy.data.r = euler(2);
    }

    m_rpyOut.write();
    m_rpyRawOut.write();
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
    Q_angle = i_param.Q_angle;
    Q_rate = i_param.Q_rate;
    R_angle = i_param.R_angle;
    r_filter.setF(1, -m_dt, 0, 1);
    r_filter.setP(0, 0, 0, 0);
    r_filter.setQ(Q_angle*m_dt, 0, 0, Q_rate*m_dt);
    r_filter.setR(R_angle);
    r_filter.setB(m_dt, 0);

    p_filter.setF(1, -m_dt, 0, 1);
    p_filter.setP(0, 0, 0, 0);
    p_filter.setQ(Q_angle*m_dt, 0, 0, Q_rate*m_dt);
    p_filter.setR(R_angle);
    p_filter.setB(m_dt, 0);

    y_filter.setF(1, -m_dt, 0, 1);
    y_filter.setP(0, 0, 0, 0);
    y_filter.setQ(Q_angle*m_dt, 0, 0, Q_rate*m_dt);
    y_filter.setR(R_angle);
    y_filter.setB(m_dt, 0);

    kf_algorithm = i_param.kf_algorithm;
    std::cerr << "[" << m_profile.instance_name << "]   Q_angle=" << Q_angle << ", Q_rate=" << Q_rate << ", R_angle=" << R_angle << std::endl;
    std::cerr << "[" << m_profile.instance_name << "]   kf_algorithm=" << (kf_algorithm==OpenHRP::KalmanFilterService::RPYKalmanFilter?"RPYKalmanFilter":"QuaternionExtendedKalmanFilter") << std::endl;
    return true;
}

bool KalmanFilter::resetKalmanFilterState()
{
    r_filter.resetStateByObservation();
    p_filter.resetStateByObservation();
    y_filter.resetStateByObservation();
};

bool KalmanFilter::getKalmanFilterParam(OpenHRP::KalmanFilterService::KalmanFilterParam& i_param)
{
  i_param.Q_angle = Q_angle;
  i_param.Q_rate = Q_rate;
  i_param.R_angle = R_angle;
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


