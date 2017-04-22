// -*- C++ -*-
/*!
 * @file  WholeBodyMasterSlave.h
 * @brief WholeBodyMasterSlave component
 * @date  $Date$
 *
 * $Id$
 */

#ifndef WholeBodyMasterSlave_H
#define WholeBodyMasterSlave_H

#include <rtm/idl/BasicDataType.hh>
#include <rtm/idl/ExtendedDataTypes.hh>
#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>
#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/idl/ExtendedDataTypesSkel.h>
#include <hrpModel/Body.h>
#include "../ImpedanceController/JointPathEx.h"
#include "../ImpedanceController/RatsMatrix.h"
// Service implementation headers
// <rtc-template block="service_impl_h">
#include "WholeBodyMasterSlaveService_impl.h"
#include "interpolator.h"
#include "../TorqueFilter/IIRFilter.h"
#include "wbms_core.h"
#include "../AutoBalancer/AutoBalancer.h"

using namespace RTC;

//static std::ostream& operator<<(std::ostream& os, const struct RTC::Time &tm)
//{
//    int pre = os.precision();
//    os.setf(std::ios::fixed);
//    os << std::setprecision(6)
//       << (tm.sec + tm.nsec/1e9)
//       << std::setprecision(pre);
//    os.unsetf(std::ios::fixed);
//    return os;
//}


class WholeBodyMasterSlave
  : public RTC::DataFlowComponentBase
{
 public:
  WholeBodyMasterSlave(RTC::Manager* manager);
  virtual ~WholeBodyMasterSlave();
  virtual RTC::ReturnCode_t onInitialize();
  virtual RTC::ReturnCode_t onFinalize();
  virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);
  virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);
  virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);
  bool startCountDownForWholeBodyMasterSlave(const double sec);
  bool stopHumanSync();
  bool setWholeBodyMasterSlaveParam(const OpenHRP::WholeBodyMasterSlaveService::WholeBodyMasterSlaveParam& i_param);
  bool getWholeBodyMasterSlaveParam(OpenHRP::WholeBodyMasterSlaveService::WholeBodyMasterSlaveParam& i_param);

 protected:
  TimedDoubleSeq m_qRef;
  InPort<TimedDoubleSeq> m_qRefIn;
  TimedPoint3D m_basePos;
  InPort<TimedPoint3D> m_basePosIn;
  TimedOrientation3D m_baseRpy;
  InPort<TimedOrientation3D> m_baseRpyIn;
  TimedPoint3D m_zmp;
  InPort<TimedPoint3D> m_zmpIn;
  TimedDoubleSeq m_optionalData;
  InPort<TimedDoubleSeq> m_optionalDataIn;

  TimedPose3D m_htcom;
  InPort<TimedPose3D> m_htcomIn;
  TimedPose3D m_htrf;
  InPort<TimedPose3D> m_htrfIn;
  TimedPose3D m_htlf;
  InPort<TimedPose3D> m_htlfIn;
  TimedPose3D m_htrh;
  InPort<TimedPose3D> m_htrhIn;
  TimedPose3D m_htlh;
  InPort<TimedPose3D> m_htlhIn;
  TimedPose3D m_hthead;
  InPort<TimedPose3D> m_htheadIn;
  TimedPoint3D m_htzmp;
  InPort<TimedPoint3D> m_htzmpIn;
  TimedPoint3D m_actzmp;
  InPort<TimedPoint3D> m_actzmpIn;
  TimedDoubleSeq m_htrfw;
  InPort<TimedDoubleSeq> m_htrfwIn;
  TimedDoubleSeq m_htlfw;
  InPort<TimedDoubleSeq> m_htlfwIn;

  TimedPose3D m_htcom_dbg;
  OutPort<TimedPose3D> m_htcom_dbgOut;
  TimedPose3D m_htrf_dbg;
  OutPort<TimedPose3D> m_htrf_dbgOut;
  TimedPose3D m_htlf_dbg;
  OutPort<TimedPose3D> m_htlf_dbgOut;
  TimedPose3D m_htrh_dbg;
  OutPort<TimedPose3D> m_htrh_dbgOut;
  TimedPose3D m_htlh_dbg;
  OutPort<TimedPose3D> m_htlh_dbgOut;
  TimedPose3D m_hthead_dbg;
  OutPort<TimedPose3D> m_hthead_dbgOut;
  TimedPoint3D m_htzmp_dbg;
  OutPort<TimedPoint3D> m_htzmp_dbgOut;
  TimedDoubleSeq m_htrfw_dbg;
  OutPort<TimedDoubleSeq> m_htrfw_dbgOut;
  TimedDoubleSeq m_htlfw_dbg;
  OutPort<TimedDoubleSeq> m_htlfw_dbgOut;
  TimedPose3D m_rpcom_dbg;
  OutPort<TimedPose3D> m_rpcom_dbgOut;
  TimedPose3D m_rprf_dbg;
  OutPort<TimedPose3D> m_rprf_dbgOut;
  TimedPose3D m_rplf_dbg;
  OutPort<TimedPose3D> m_rplf_dbgOut;
  TimedPose3D m_rprh_dbg;
  OutPort<TimedPose3D> m_rprh_dbgOut;
  TimedPose3D m_rplh_dbg;
  OutPort<TimedPose3D> m_rplh_dbgOut;
  TimedPose3D m_rphead_dbg;
  OutPort<TimedPose3D> m_rphead_dbgOut;
  TimedPoint3D m_rpzmp_dbg;
  OutPort<TimedPoint3D> m_rpzmp_dbgOut;
  TimedPoint3D m_rpdcp_dbg;
  OutPort<TimedPoint3D> m_rpdcp_dbgOut;
  TimedPoint3D m_rpacp_dbg;
  OutPort<TimedPoint3D> m_rpacp_dbgOut;
  TimedDoubleSeq m_invdyn_dbg;
  OutPort<TimedDoubleSeq> m_invdyn_dbgOut;

  OutPort<TimedDoubleSeq> m_qOut;
  RTC::OutPort<RTC::TimedPoint3D> m_zmpOut;
  OutPort<TimedPoint3D> m_basePosOut;
  OutPort<TimedOrientation3D> m_baseRpyOut;
  OutPort<TimedDoubleSeq> m_optionalDataOut;
  RTC::CorbaPort m_WholeBodyMasterSlaveServicePort;

  WholeBodyMasterSlaveService_impl m_service0;

 private:
  struct ABCIKparam {
    hrp::Vector3 target_p0, localPos, adjust_interpolation_target_p0, adjust_interpolation_org_p0;
    hrp::Matrix33 target_r0, localR, adjust_interpolation_target_r0, adjust_interpolation_org_r0;
    hrp::Link* target_link;
    bool is_active, has_toe_joint;
  };
  void solveFullbodyIKStrictCOM(const HRPPose3D& com_ref, const HRPPose3D& rf_ref, const HRPPose3D& lf_ref, const HRPPose3D& rh_ref, const HRPPose3D& lh_ref, const hrp::Vector3& head_ref);
  void processWholeBodyMasterSlave();
  void calcDynamicsFilterCompensation(const hrp::Vector3 zmp_lip, const hrp::Vector3 zmp_fullbody);
  bool isOptionalDataContact (const std::string& ee_name)
  {
      return (std::fabs(m_optionalData.data[contact_states_index_map[ee_name]]-1.0)<0.1)?true:false;
  };
  typedef boost::shared_ptr<SimpleFullbodyInverseKinematicsSolver> fikPtr;
  fikPtr fik;
  std::map<std::string, size_t> contact_states_index_map;
  double m_dt;
  hrp::BodyPtr m_robot;
  hrp::Vector3 input_zmp, input_basePos;
  hrp::Matrix33 input_baseRot;

  hrp::Vector3 rel_ref_zmp; // ref zmp in base frame

  unsigned int m_debugLevel;
  bool is_legged_robot;// is_stop_mode, is_hand_fix_mode, is_hand_fix_initial;
  int loop;

  hrp::InvDynStateBuffer idsb;
  std::vector<IIRFilter> invdyn_zmp_filters;

  hrp::InvDynStateBuffer idsb2;
  std::vector<IIRFilter> invdyn_zmp_filters2;

  hrp::Vector3 ref_zmp_invdyn2;

  boost::shared_ptr<HumanSynchronizer> hsp;
};

extern "C"
{
  void WholeBodyMasterSlaveInit(RTC::Manager* manager);
};

#endif // WholeBodyMasterSlave_H
