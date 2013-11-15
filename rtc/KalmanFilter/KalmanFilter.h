// -*- C++ -*-
/*!
 * @file  KalmanFilter.h
 * @brief null component
 * @date  $Date$
 *
 * $Id$
 */

#ifndef NULL_COMPONENT_H
#define NULL_COMPONENT_H

#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>
#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/idl/ExtendedDataTypesSkel.h>
#include <hrpModel/Body.h>

#include <hrpUtil/EigenTypes.h>
namespace hrp{
    typedef Eigen::Vector2d Vector2;
    typedef Eigen::Matrix2d Matrix22;
}

class KFilter {
public:
  KFilter() {
      F(0,0) = 1; F(0,1) =-0.005;
      F(1,0) = 0; F(1,1) =    1;

      P(0,0) = 0; P(0,1) =    0;
      P(1,0) = 0; P(1,1) =    0;

      Q(0,0) = 0.001 * 0.005; Q(0,1) = 0.000 * 0.005;
      Q(1,0) = 0.000 * 0.005; Q(1,1) = 0.003 * 0.005;

      R = 0.03;

      B(0) = 0.005; B(1) = 0;

      H(0,0) = 1; H(0,1) = 0;

      x(0) = 0; x(1) = 0;

      I = hrp::dmatrix::Identity(2,2);
  }
  void setF(double _f0, double _f1, double _f2, double _f3) { F(0,0) = _f0; F(0,1) = _f1; F(1,0) = _f2; F(1,1) = _f3;}
  void setP(double _p0, double _p1, double _p2, double _p3) { P(0,0) = _p0; P(0,1) = _p1; P(1,0) = _p2; P(1,1) = _p3;}
  void setQ(double _q0, double _q1, double _q2, double _q3) { Q(0,0) = _q0; Q(0,1) = _q1; Q(1,0) = _q2; Q(1,1) = _q3;}
  void setR(double _R) { R = _R; }
  void setB(double _b0, double _b1) { B[0] = _b0; B[1] = _b1; }
  hrp::Vector2 &getx() { return x; }
  void update(double u, double z) {
      // Predicted (a priori) state estimate
      x = F * x + B * u;
      // Predicted (a priori) estimate covariance
      P = F * P * F.transpose() + Q;
      //
      // Innovation or measurement residual
      double y = z - H * x;
      // Innovation (or residual) covariance
      double S = H * P * H.transpose() + R;
      // Optimal Kalman gain
      K = P * H.transpose() / S;
      // Updated (a posteriori) state estimate
      x = x + K * y;
      // Updated (a posteriori) estimate covariance
      P = (I - K * H) * P;
  }
private:
  hrp::Matrix22 P, Q, I, F;
  Eigen::Matrix<double, 1, 2> H;
  hrp::Vector2 B, K;
  hrp::Vector2 x;
  double R;
};

// Service implementation headers
// <rtc-template block="service_impl_h">
#include "KalmanFilterService_impl.h"

// </rtc-template>

// Service Consumer stub headers
// <rtc-template block="consumer_stub_h">

// </rtc-template>

using namespace RTC;

/**
   \brief sample RT component which has one data input port and one data output port
 */
class KalmanFilter
  : public RTC::DataFlowComponentBase
{
 public:
  /**
     \brief Constructor
     \param manager pointer to the Manager
  */
  KalmanFilter(RTC::Manager* manager);
  /**
     \brief Destructor
  */
  virtual ~KalmanFilter();

  // The initialize action (on CREATED->ALIVE transition)
  // formaer rtc_init_entry()
  virtual RTC::ReturnCode_t onInitialize();

  // The finalize action (on ALIVE->END transition)
  // formaer rtc_exiting_entry()
  // virtual RTC::ReturnCode_t onFinalize();

  // The startup action when ExecutionContext startup
  // former rtc_starting_entry()
  // virtual RTC::ReturnCode_t onStartup(RTC::UniqueId ec_id);

  // The shutdown action when ExecutionContext stop
  // former rtc_stopping_entry()
  // virtual RTC::ReturnCode_t onShutdown(RTC::UniqueId ec_id);

  // The activated action (Active state entry action)
  // former rtc_active_entry()
  virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);

  // The deactivated action (Active state exit action)
  // former rtc_active_exit()
  virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);

  // The execution action that is invoked periodically
  // former rtc_active_do()
  virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

  // The aborting action when main logic error occurred.
  // former rtc_aborting_entry()
  // virtual RTC::ReturnCode_t onAborting(RTC::UniqueId ec_id);

  // The error action in ERROR state
  // former rtc_error_do()
  // virtual RTC::ReturnCode_t onError(RTC::UniqueId ec_id);

  // The reset action that is invoked resetting
  // This is same but different the former rtc_init_entry()
  // virtual RTC::ReturnCode_t onReset(RTC::UniqueId ec_id);

  // The state update action that is invoked after onExecute() action
  // no corresponding operation exists in OpenRTm-aist-0.2.0
  // virtual RTC::ReturnCode_t onStateUpdate(RTC::UniqueId ec_id);

  // The action that is invoked when execution context's rate is changed
  // no corresponding operation exists in OpenRTm-aist-0.2.0
  // virtual RTC::ReturnCode_t onRateChanged(RTC::UniqueId ec_id);

  bool SetKalmanFilterParam(double Q_angle, double Q_rate, double R_angle);

 protected:
  // Configuration variable declaration
  // <rtc-template block="config_declare">
  
  // </rtc-template>

  TimedAngularVelocity3D m_rate;
  TimedAcceleration3D m_acc;
  TimedAcceleration3D m_accRef;
  TimedOrientation3D m_rpy;
  TimedOrientation3D m_rpyRaw;
  TimedOrientation3D m_rpy_prev;
  TimedOrientation3D m_rpyRaw_prev;

  // DataInPort declaration
  // <rtc-template block="inport_declare">
  InPort<TimedAngularVelocity3D> m_rateIn;
  InPort<TimedAcceleration3D> m_accIn;
  InPort<TimedAcceleration3D> m_accRefIn;
  InPort<TimedAngularVelocity3D> m_rpyIn; // for dummy usage
  
  // </rtc-template>

  // DataOutPort declaration
  // <rtc-template block="outport_declare">
  OutPort<TimedOrientation3D> m_rpyOut;
  OutPort<TimedOrientation3D> m_rpyRawOut;
  
  // </rtc-template>

  // CORBA Port declaration
  // <rtc-template block="corbaport_declare">
  
  // </rtc-template>

  // Service declaration
  // <rtc-template block="service_declare">
  RTC::CorbaPort m_KalmanFilterServicePort;
  
  // </rtc-template>

  // Consumer declaration
  // <rtc-template block="consumer_declare">
  KalmanFilterService_impl m_service0;
  
  // </rtc-template>

 private:
  double m_dt;
  KFilter r_filter, p_filter, y_filter;
  hrp::BodyPtr m_robot;
  hrp::Matrix33 m_sensorR;
  unsigned int m_debugLevel;
  int dummy;
};


extern "C"
{
  void KalmanFilterInit(RTC::Manager* manager);
};

#endif // NULL_COMPONENT_H
