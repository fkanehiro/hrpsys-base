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
// Service implementation headers
// <rtc-template block="service_impl_h">
#include "WholeBodyMasterSlaveService_impl.h"
#include "interpolator.h"
#include "../TorqueFilter/IIRFilter.h"
#include "wbms_core.h"
//#include "../AutoBalancer/AutoBalancer.h"

using namespace RTC;

//#define USE_DEBUG_PORT

enum mode_enum{ MODE_IDLE, MODE_SYNC_TO_WBMS, MODE_WBMS, MODE_PAUSE, MODE_SYNC_TO_IDLE};

class ControlMode{
    private:
        mode_enum current_mode, previous_mode, requested_mode;
    public:
        ControlMode(){ current_mode = previous_mode = requested_mode = MODE_IDLE;}
        ~ControlMode(){}
        bool setModeRequest(mode_enum in){
            switch(in){
                case MODE_SYNC_TO_WBMS:
                    if(current_mode == MODE_IDLE){ requested_mode = MODE_SYNC_TO_WBMS; return true; }else{ return false; }
                case MODE_WBMS:
                    if(current_mode == MODE_SYNC_TO_WBMS || current_mode == MODE_PAUSE ){ requested_mode = MODE_WBMS; return true; }else{ return false; }
                case MODE_PAUSE:
                    if(current_mode == MODE_WBMS){ requested_mode = MODE_PAUSE; return true; }else{ return false; }
                case MODE_SYNC_TO_IDLE:
                    if(current_mode == MODE_WBMS || current_mode == MODE_PAUSE ){ requested_mode = MODE_SYNC_TO_IDLE; return true; }else{ return false; }
                case MODE_IDLE:
                    if(current_mode == MODE_SYNC_TO_IDLE ){ requested_mode = MODE_IDLE; return true; }else{ return false; }
                default:
                    return false;
            }
        }
        void update(){ previous_mode = current_mode; current_mode = requested_mode; }
        mode_enum now(){ return current_mode; }
        mode_enum pre(){ return previous_mode; }
        bool isRunning(){ return (current_mode==MODE_SYNC_TO_WBMS) || (current_mode==MODE_WBMS) || (current_mode==MODE_PAUSE) || (current_mode==MODE_SYNC_TO_IDLE) ;}
        bool isInitialize(){ return (previous_mode==MODE_IDLE) && (current_mode==MODE_SYNC_TO_WBMS) ;}
};

class WholeBodyMasterSlave : public RTC::DataFlowComponentBase, UTIL_CONST {
    public:
    WholeBodyMasterSlave(RTC::Manager* manager);
    virtual ~WholeBodyMasterSlave();
    virtual RTC::ReturnCode_t onInitialize();
    virtual RTC::ReturnCode_t onFinalize();
    virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);
    virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);
    virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);
    bool startCountDownForWholeBodyMasterSlave(const double sec);
    bool startWholeBodyMasterSlave();
    bool stopWholeBodyMasterSlave();
    bool pauseWholeBodyMasterSlave();
    bool resumeWholeBodyMasterSlave();
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
    //  TimedPoint3D m_actzmp;
    //  InPort<TimedPoint3D> m_actzmpIn;
    TimedDoubleSeq m_htrfw;
    InPort<TimedDoubleSeq> m_htrfwIn;
    TimedDoubleSeq m_htlfw;
    InPort<TimedDoubleSeq> m_htlfwIn;

#ifdef USE_DEBUG_PORT
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
#endif

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
    //  typedef boost::shared_ptr<SimpleFullbodyInverseKinematicsSolver> fikPtr;
    //  typedef boost::shared_ptr<FullbodyInverseKinematicsSolver> fikPtr;
    typedef boost::shared_ptr<FullbodyInverseKinematicsSolver> fikPtr;
    fikPtr fik, fik_ml;
    hrp::BodyPtr m_robot, m_robot_ml, m_robot_vsafe;
    std::vector<fikPtr> fik_list;
    std::vector<hrp::BodyPtr> body_list;
    std::map<std::string, IKConstraint> eename_ikcp_map;
    std::map<std::string, size_t> contact_states_index_map;
    double m_dt;

    double transition_interpolator_ratio;
    interpolator *transition_interpolator;
    int ROBOT_ALL_DOF;
    interpolator *q_ip;
    hrp::dvector init_sync_state;

    int optionalDataLength;
    unsigned int m_debugLevel;
    bool is_legged_robot;
    unsigned int loop;

    double avg_q_vel;

    hrp::InvDynStateBuffer idsb;
    BiquadIIRFilterVec invdyn_zmp_filters;

    BiquadIIRFilterVec final_ref_zmp_filter;

    IIRFilter ref_ee_vel_v_filter;

    HumanPose raw_pose;

    boost::shared_ptr<WBMSCore> hsp;
    boost::shared_ptr<CapsuleCollisionChecker> sccp;

    hrp::Vector3 torso_rot_rmc;
    ControlMode mode;

    //  enum { MODE_IDLE, MODE_SYNC_TO_WBMS, MODE_WBMS, MODE_PAUSE, MODE_SYNC_TO_IDLE} mode, previous_mode;
    //  bool isRunning(){ return (mode==MODE_SYNC_TO_WBMS) || (mode==MODE_WBMS) || (mode==MODE_PAUSE) || (mode==MODE_SYNC_TO_IDLE) ;}
    //  bool isInitialize(){ return (previous_mode==MODE_IDLE) && (mode==MODE_SYNC_TO_WBMS) ;}
    //  enum mode_enum{ MODE_IDLE, MODE_SYNC_TO_WBMS, MODE_WBMS, MODE_PAUSE, MODE_SYNC_TO_IDLE};

    void setupfik(fikPtr& fik_in, hrp::BodyPtr& robot_in, RTC::Properties& prop_in);
    void calcManipulability(fikPtr& fik_in, hrp::BodyPtr& robot_in);
    void calcManipulabilityJointLimit(fikPtr& fik_in, hrp::BodyPtr& robot_in, hrp::Vector3 target_v_old[], WBMSPose3D& rf_ref, WBMSPose3D& lf_ref, WBMSPose3D& rh_ref, WBMSPose3D& lh_ref);
    void calcManipulabilityJointLimitForWBMS(fikPtr& fik_in, hrp::BodyPtr& robot_in);
    void solveFullbodyIKStrictCOM(fikPtr& fik_in, hrp::BodyPtr& robot_in, const WBMSPose3D& com_ref, const WBMSPose3D& rf_ref, const WBMSPose3D& lf_ref, const WBMSPose3D& rh_ref, const WBMSPose3D& lh_ref, const WBMSPose3D& head_ref, const std::string& debug_prefix="");
    void processTransition();
    void preProcessForWholeBodyMasterSlave(fikPtr& fik_in, hrp::BodyPtr& robot_in);
    void processWholeBodyMasterSlave(fikPtr& fik_in, hrp::BodyPtr& robot_in, const HumanPose& pose_ref);
    void processWholeBodyMasterSlave_Raw(fikPtr& fik_in, hrp::BodyPtr& robot_in, HumanPose& pose_ref);
    void processMomentumCompensation(fikPtr& fik_in, hrp::BodyPtr& robot_in, hrp::BodyPtr& robot_normal_in, const HumanPose& pose_ref);
    void processHOFFARBIBFilter(hrp::BodyPtr& robot_in, hrp::BodyPtr& robot_out);
    bool isOptionalDataContact (const std::string& ee_name) { return (std::fabs(m_optionalData.data[contact_states_index_map[ee_name]]-1.0)<0.1)?true:false; };

    //  void calcVelAccSafeTrajectory(const hrp::Vector3& pos_cur, const hrp::Vector3& vel_cur, const hrp::Vector3& pos_tgt, const hrp::Vector3& max_acc, const double& max_vel, hrp::Vector3& pos_ans);
};






extern "C"
{
    void WholeBodyMasterSlaveInit(RTC::Manager* manager);
};

#endif // WholeBodyMasterSlave_H
