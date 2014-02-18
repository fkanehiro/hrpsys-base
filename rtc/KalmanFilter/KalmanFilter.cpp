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
    dummy(0)
{
    m_service0.kalman(this);
}

KalmanFilter::~KalmanFilter()
{
}



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
      std::cerr << "KalmanFilter failed to prop[dt] " << prop["dt"] << "" 
                << std::endl;
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
      std::cerr << "failed to load model[" << prop["model"] << "]" 
                << std::endl;
  }

  r_filter.setF(1, -m_dt, 0, 1);
  r_filter.setP(0, 0, 0, 0);
  r_filter.setQ(0.001*m_dt, 0, 0, 0.003*m_dt);
  r_filter.setR(0.03);
  r_filter.setB(m_dt, 0);

  p_filter.setF(1, -m_dt, 0, 1);
  p_filter.setP(0, 0, 0, 0);
  p_filter.setQ(0.001*m_dt, 0, 0, 0.003*m_dt);
  p_filter.setR(0.03);
  p_filter.setB(m_dt, 0);

  y_filter.setF(1, -m_dt, 0, 1);
  y_filter.setP(0, 0, 0, 0);
  y_filter.setQ(0.001*m_dt, 0, 0, 0.003*m_dt);
  y_filter.setR(0.03);
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
  if (m_accRefIn.isNew()){
      m_accRefIn.read();
  }
  if (m_accIn.isNew()){
      m_accIn.read();

      //std::cerr << "rate : " << m_rate.data.avx <<  " " << m_rate.data.avy <<  " " << m_rate.data.avz << std::endl; // rad/sec (AngularVelocity3D) / gyro / stable, drift
      //std::cerr << "acc  : " << m_acc.data.ax   <<  " " << m_acc.data.ay   <<  " " << m_acc.data.az   << std::endl; //   m/sec (Acceleration3D) / accelerometer / unstable , no drift
      //
      // G = [ cosb, sinb sina, sinb cosa,
      //          0,      cosa,     -sina,
      //      -sinb, cosb sina, cosb cosa]
      // s = [sx, sy, sz]t ( accelerometer )
      // g = [ 0 0 -g]t
      // s = G g = [g sinb, -g cosb sina, -g cosb cosa]t
      double sx = m_acc.data.ax, sy = m_acc.data.ay, sz = m_acc.data.az;

      // hrp::RateGyroSensor* sensor = m_robot->sensor<hrp::RateGyroSensor>("gyrometer");
      hrp::Vector3 s = m_sensorR * hrp::Vector3(sx, sy, sz);
      sx = s(0); sy = s(1); sz = s(2); // transform to imaginary acc data
      double g = sqrt(sx * sx + sy * sy + sz * sz);
      // atan2(y, x) = atan(y/x)
      double a, b;
#if 0
      x/g = sinb
     -y/g = cosb sina
     -z/g = cosb cosa
      y/g^2 = cosb^2 sina^2
      z/g^2 = cosb^2 cosa^2
      y/g^2 + z/g^2 = cosb^2
      y/g^2 = (1 - sinb^2) sina^2
      z/g^2 = (1 - sinb^2) cosa^2
      y/g^2 = (1 - x/g^2) sina^2
      z/g^2 = (1 - x/g^2) cosa^2
#endif
      b = atan2( - sx / g, sqrt( sy/g * sy/g + sz/g * sz/g ) );
      a = atan2( ( sy/g ) / sqrt( 1 - sx/g * sx/g),
                 ( sz/g ) / sqrt( 1 - sx/g * sx/g) );
      //std::cerr << "a(roll) = " << a*180/M_PI << ", b(pitch) = " << b*180/M_PI << ",  sx = " << sx << ", sy = " << sy << ", sz = " << sz << std::endl;
      m_rpyRaw.data.r = a;
      m_rpyRaw.data.p = b;
      m_rpyRaw.data.y = 0;
#if 0
      m_rpy.data.r = m_rpyRaw.data.r;
      m_rpy.data.p = m_rpyRaw.data.p;
      m_rpy.data.y = m_rpyRaw.data.y;
#endif
#if 0
      // complementary filter
      m_rpy.data.r = 0.98 *(m_rpy.data.r+m_rate.data.avx*m_dt) + 0.02*m_rpyRaw.data.r;
      m_rpy.data.p = 0.98 *(m_rpy.data.p+m_rate.data.avy*m_dt) + 0.02*m_rpyRaw.data.p;
      m_rpy.data.y = 0.98 *(m_rpy.data.y+m_rate.data.avz*m_dt) + 0.02*m_rpyRaw.data.y;
#endif
      // kalman filter
      // x[0] = m_rpyRaw.data.r; // angle (from m/sec Acceleration3D, unstable, no drift )
      // x[1] = m_rate.data.avx; // rate ( rad/sec, AngularVelocity, gyro, stable/drift )
      hrp::Vector3 av = m_sensorR * hrp::Vector3(m_rate.data.avx, m_rate.data.avy, m_rate.data.avz);
      m_rate.data.avx = av(0); m_rate.data.avy = av(1); m_rate.data.avz = av(2); // transform to imaginary rate data
      // use kalman filter with imaginary data
      r_filter.update(m_rate.data.avx, m_rpyRaw.data.r);
      p_filter.update(m_rate.data.avy, m_rpyRaw.data.p);
      y_filter.update(m_rate.data.avz, m_rpyRaw.data.y);

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
#if 0
      std::cout << m_acc.tm.sec + m_acc.tm.nsec/1000000000.0 << " "
                << m_rpyRaw.data.r << " "
                << m_rpyRaw.data.p << " "
                << m_rpyRaw.data.y << " "
                << m_rpy.data.r << " "
                << m_rpy.data.p << " "
                << m_rpy.data.y << " "
                << (m_rpyRaw.data.r - m_rpyRaw_prev.data.r) << " "
                << (m_rpyRaw.data.p - m_rpyRaw_prev.data.p) << " "
                << (m_rpyRaw.data.y - m_rpyRaw_prev.data.y) << " "
                << (m_rpy.data.r - m_rpy_prev.data.r) << " "
                << (m_rpy.data.p - m_rpy_prev.data.p) << " "
                << (m_rpy.data.y - m_rpy_prev.data.y)
                << std::endl;
      m_rpy_prev = m_rpy;
      m_rpyRaw_prev = m_rpyRaw;
#endif

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

bool KalmanFilter::SetKalmanFilterParam(double Q_angle, double Q_rate, double R_angle) {

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


