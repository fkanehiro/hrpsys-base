// -*- C++ -*-
/*!
 * @file  ImpedanceController.h
 * @brief impedance control component
 * @date  $Date$
 *
 * $Id$
 */

#ifndef IMPEDANCE_H
#define IMPEDANCE_H

#include <boost/interprocess/sync/interprocess_semaphore.hpp>
#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>
#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/idl/ExtendedDataTypesSkel.h>
#include <hrpModel/Body.h>
#include "JointPathEx.h"
// Service implementation headers
// <rtc-template block="service_impl_h">
#include "ImpedanceControllerService_impl.h"

// </rtc-template>

// Service Consumer stub headers
// <rtc-template block="consumer_stub_h">

// </rtc-template>

using namespace RTC;

class ImpedanceController
  : public RTC::DataFlowComponentBase
{
 public:
  ImpedanceController(RTC::Manager* manager);
  virtual ~ImpedanceController();

  // The initialize action (on CREATED->ALIVE transition)
  // formaer rtc_init_entry()
 virtual RTC::ReturnCode_t onInitialize();

  // The finalize action (on ALIVE->END transition)
  // formaer rtc_exiting_entry()
  virtual RTC::ReturnCode_t onFinalize();

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
  // virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);

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

  bool setImpedanceControllerParam(OpenHRP::ImpedanceControllerService::impedanceParam i_param_);
  bool getImpedanceControllerParam(const std::string& i_name_, OpenHRP::ImpedanceControllerService::impedanceParam& i_param_);
  bool deleteImpedanceController(std::string i_name_);
  void waitDeletingImpedanceController(std::string i_name_);
  bool deleteImpedanceControllerAndWait(std::string i_name_);

 protected:
  // Configuration variable declaration
  // <rtc-template block="config_declare">
  
  // </rtc-template>

  // DataInPort declaration
  // <rtc-template block="inport_declare">
  TimedDoubleSeq m_qCurrent;
  InPort<TimedDoubleSeq> m_qCurrentIn;
  TimedDoubleSeq m_qRef;
  InPort<TimedDoubleSeq> m_qRefIn;
  std::vector<TimedDoubleSeq> m_force;
  std::vector<InPort<TimedDoubleSeq> *> m_forceIn;
  
  // </rtc-template>

  // DataOutPort declaration
  // <rtc-template block="outport_declare">
  TimedDoubleSeq m_q;
  OutPort<TimedDoubleSeq> m_qOut;
  
  // </rtc-template>

  // CORBA Port declaration
  // <rtc-template block="corbaport_declare">
  RTC::CorbaPort m_ImpedanceControllerServicePort;

  // </rtc-template>

  // Service declaration
  // <rtc-template block="service_declare">
  ImpedanceControllerService_impl m_service0;

  // </rtc-template>

  // Consumer declaration
  // <rtc-template block="consumer_declare">
  
  // </rtc-template>

 private:
  struct ImpedanceParam{
    std::string base_name, target_name;
    hrp::Vector3 target_p0, target_p1, current_p0, current_p1, current_p2;
    hrp::Matrix33 target_r0, target_r1, current_r0, current_r1, current_r2;
    hrp::Vector3 force_offset_p, force_offset_r;
    double M_p, D_p, K_p;
    double M_r, D_r, K_r;
    hrp::Vector3 ref_force, ref_moment;
    hrp::Matrix33 force_gain, moment_gain;
    double sr_gain, avoid_gain, reference_gain, manipulability_limit;
    int transition_count; // negative value when initing and positive value when deleting
    hrp::dvector transition_joint_q;
    hrp::JointPathExPtr manip;

    ImpedanceParam ()
      : M_p(10), D_p(200), K_p(400), M_r(5), D_r(100), K_r(200),
        ref_force(hrp::Vector3::Zero()), ref_moment(hrp::Vector3::Zero()),
        force_gain(hrp::Matrix33::Identity()), moment_gain(hrp::Matrix33::Identity()),
        sr_gain(1.0), avoid_gain(0.001), reference_gain(0.01), manipulability_limit(0.1)
    {};
  };
  struct VirtualForceSensorParam {
    hrp::Vector3 p;
    hrp::Matrix33 R;
  };

  // for calculation of rotation
  void calcDifferenceRotation(hrp::Vector3& ret_dif_rot, const hrp::Matrix33& self_rot, const hrp::Matrix33& target_rot)
  {
    //ret_dif_rot = self_rot * hrp::omegaFromRot(self_rot.transpose() * target_rot);
    ret_dif_rot = self_rot * hrp::Vector3(matrix_log(hrp::Matrix33(self_rot.transpose() * target_rot)));
  }

  hrp::Vector3 matrix_log(const hrp::Matrix33& m) {
    hrp::Vector3 mlog;
    double q0, th;
    hrp::Vector3 q;
    double norm;
  
    Eigen::Quaternion<double> eiq(m);
    q0 = eiq.w();
    q = eiq.vec();
    norm = q.norm();
    if (norm > 0) {
      if ((q0 > 1.0e-10) || (q0 < -1.0e-10)) {
	th = 2 * std::atan(norm / q0);
      } else if (q0 > 0) {
	th = M_PI / 2;
      } else {
	th = -M_PI / 2;
      }
      mlog = (th / norm) * q ;
    } else {
      mlog = hrp::Vector3::Zero();
    }
    return mlog;
  }

  // matrix product using quaternion normalization
  void rotm3times (hrp::Matrix33& m12, const hrp::Matrix33& m1, const hrp::Matrix33& m2) {
    Eigen::Quaternion<double> eiq1(m1);
    Eigen::Quaternion<double> eiq2(m2);
    Eigen::Quaternion<double> eiq3;
    eiq3 = eiq1 * eiq2;
    eiq3.normalize();
    m12 = eiq3.toRotationMatrix();
  }

  // copied from JSK codes ;; hrp::rodorigues(vel_r.norm(), vel_r.normalized()) does not work because of nan?
  void rotation_matrix(hrp::Matrix33& rm, const double theta, const hrp::Vector3& axis) {
    double cs, sn, vers, xv, yv, zv, xyv, yzv, zxv, xs, ys, zs;
    hrp::Vector3 a;

    (cs = std::cos(theta));
    (sn = std::sin(theta));
    (vers = (1 - cs));
    if (axis.norm() > 0) {
      a = axis.normalized();
    } else {
      a = hrp::Vector3(0,0,0);
    }
    (xv = ((a(0) * a(0)) * vers));
    (yv = ((a(1) * a(1)) * vers));
    (zv = ((a(2) * a(2)) * vers));
    (xyv = ((a(0) * a(1)) * vers));
    (yzv = ((a(1) * a(2)) * vers));
    (zxv = ((a(2) * a(0)) * vers));
    (xs = (a(0) * sn));
    (ys = (a(1) * sn));
    (zs = (a(2) * sn));
    (rm(0, 0) = (xv + cs));
    (rm(0, 1) = (xyv - zs));
    (rm(0, 2) = (zxv + ys));
    (rm(1, 0) = (xyv + zs));
    (rm(1, 1) = (yv + cs));
    (rm(1, 2) = (yzv - xs));
    (rm(2, 0) = (zxv - ys));
    (rm(2, 1) = (yzv + xs));
    (rm(2, 2) = (zv + cs));
  }

  inline hrp::Matrix33 rotation_matrix(const double theta, const hrp::Vector3& axis) {
    hrp::Matrix33 ret;
    rotation_matrix(ret, theta, axis);
    return ret;
  }

  bool checkImpedanceNameValidity (int& force_id, const std::string& name);

  void copyImpedanceParam (OpenHRP::ImpedanceControllerService::impedanceParam& i_param_, const ImpedanceParam& param);

  std::map<std::string, ImpedanceParam> m_impedance_param;
  std::map<std::string, VirtualForceSensorParam> m_sensors;
  double m_dt;
  hrp::BodyPtr m_robot;
  coil::Mutex m_mutex;
  unsigned int m_debugLevel;
  int dummy;
};


extern "C"
{
  void ImpedanceControllerInit(RTC::Manager* manager);
};

#endif // IMPEDANCE_H
