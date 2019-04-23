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
//#include "../ImpedanceController/JointPathEx.h"
// Service implementation headers
#include "WholeBodyMasterSlaveService_impl.h"
#include "interpolator.h"
//#include "../TorqueFilter/IIRFilter.h"
#include "wbms_core.h"

//using namespace RTC;

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
    bool startWholeBodyMasterSlave();
    bool stopWholeBodyMasterSlave();
    bool pauseWholeBodyMasterSlave();
    bool resumeWholeBodyMasterSlave();
    bool setWholeBodyMasterSlaveParam(const OpenHRP::WholeBodyMasterSlaveService::WholeBodyMasterSlaveParam& i_param);
    bool getWholeBodyMasterSlaveParam(OpenHRP::WholeBodyMasterSlaveService::WholeBodyMasterSlaveParam& i_param);

    protected:
    RTC::TimedDoubleSeq m_qRef;
    RTC::InPort<RTC::TimedDoubleSeq> m_qRefIn;
    RTC::TimedPoint3D m_basePos;
    RTC::InPort<RTC::TimedPoint3D> m_basePosIn;
    RTC::TimedOrientation3D m_baseRpy;
    RTC::InPort<RTC::TimedOrientation3D> m_baseRpyIn;
    RTC::TimedPoint3D m_zmp;
    RTC::InPort<RTC::TimedPoint3D> m_zmpIn;
    RTC::TimedDoubleSeq m_optionalData;
    RTC::InPort<RTC::TimedDoubleSeq> m_optionalDataIn;

    RTC::TimedPose3D m_htcom;
    RTC::InPort<RTC::TimedPose3D> m_htcomIn;
    RTC::TimedPose3D m_htrf;
    RTC::InPort<RTC::TimedPose3D> m_htrfIn;
    RTC::TimedPose3D m_htlf;
    RTC::InPort<RTC::TimedPose3D> m_htlfIn;
    RTC::TimedPose3D m_htrh;
    RTC::InPort<RTC::TimedPose3D> m_htrhIn;
    RTC::TimedPose3D m_htlh;
    RTC::InPort<RTC::TimedPose3D> m_htlhIn;
    RTC::TimedPose3D m_hthead;
    RTC::InPort<RTC::TimedPose3D> m_htheadIn;
    RTC::TimedPoint3D m_htzmp;
    RTC::InPort<RTC::TimedPoint3D> m_htzmpIn;
    OpenHRP::TimedWrench m_htrfw;
    RTC::InPort<OpenHRP::TimedWrench> m_htrfwIn;
    OpenHRP::TimedWrench m_htlfw;
    RTC::InPort<OpenHRP::TimedWrench> m_htlfwIn;

#ifdef USE_DEBUG_PORT
    TimedPose3D m_htcom_dbg;
    RTC::OutPort<RTC::TimedPose3D> m_htcom_dbgOut;
    TimedPose3D m_htrf_dbg;
    RTC::OutPort<RTC::TimedPose3D> m_htrf_dbgOut;
    TimedPose3D m_htlf_dbg;
    RTC::OutPort<RTC::TimedPose3D> m_htlf_dbgOut;
    TimedPose3D m_htrh_dbg;
    RTC::OutPort<RTC::TimedPose3D> m_htrh_dbgOut;
    TimedPose3D m_htlh_dbg;
    RTC::OutPort<RTC::TimedPose3D> m_htlh_dbgOut;
    TimedPose3D m_hthead_dbg;
    RTC::OutPort<RTC::TimedPose3D> m_hthead_dbgOut;
    TimedPoint3D m_htzmp_dbg;
    RTC::OutPort<RTC::TimedPoint3D> m_htzmp_dbgOut;
    TimedDoubleSeq m_htrfw_dbg;
    RTC::OutPort<RTC::TimedDoubleSeq> m_htrfw_dbgOut;
    TimedDoubleSeq m_htlfw_dbg;
    RTC::OutPort<RTC::TimedDoubleSeq> m_htlfw_dbgOut;
    TimedPose3D m_rpcom_dbg;
    RTC::OutPort<RTC::TimedPose3D> m_rpcom_dbgOut;
    TimedPose3D m_rprf_dbg;
    RTC::OutPort<RTC::TimedPose3D> m_rprf_dbgOut;
    TimedPose3D m_rplf_dbg;
    RTC::OutPort<RTC::TimedPose3D> m_rplf_dbgOut;
    TimedPose3D m_rprh_dbg;
    RTC::OutPort<RTC::TimedPose3D> m_rprh_dbgOut;
    TimedPose3D m_rplh_dbg;
    RTC::OutPort<RTC::TimedPose3D> m_rplh_dbgOut;
    TimedPose3D m_rphead_dbg;
    RTC::OutPort<RTC::TimedPose3D> m_rphead_dbgOut;
    TimedPoint3D m_rpzmp_dbg;
    RTC::OutPort<RTC::TimedPoint3D> m_rpzmp_dbgOut;
    TimedPoint3D m_rpdcp_dbg;
    RTC::OutPort<RTC::TimedPoint3D> m_rpdcp_dbgOut;
    TimedPoint3D m_rpacp_dbg;
    RTC::OutPort<RTC::TimedPoint3D> m_rpacp_dbgOut;
    TimedDoubleSeq m_invdyn_dbg;
    RTC::OutPort<RTC::TimedDoubleSeq> m_invdyn_dbgOut;
#endif

    RTC::OutPort<RTC::TimedDoubleSeq> m_qOut;
    RTC::OutPort<RTC::TimedPoint3D> m_zmpOut;
    RTC::OutPort<RTC::TimedPoint3D> m_basePosOut;
    RTC::OutPort<RTC::TimedOrientation3D> m_baseRpyOut;
    RTC::OutPort<RTC::TimedDoubleSeq> m_optionalDataOut;
    RTC::CorbaPort m_WholeBodyMasterSlaveServicePort;

    WholeBodyMasterSlaveService_impl m_service0;

    private:
    typedef boost::shared_ptr<FullbodyInverseKinematicsSolver> fikPtr;
    fikPtr fik;
    hrp::BodyPtr m_robot, m_robot_vsafe;
    std::vector<hrp::BodyPtr> body_list;
    std::map<std::string, IKConstraint> ee_name_ikcp_map;
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

    boost::shared_ptr<WBMSCore> wbms;
    boost::shared_ptr<CapsuleCollisionChecker> sccp;

    hrp::Vector3 torso_rot_rmc;
    ControlMode mode;

    RTC::ReturnCode_t setupfik(fikPtr& fik_in, hrp::BodyPtr& robot_in, RTC::Properties& prop_in);
    void calcManipulability(fikPtr& fik_in, hrp::BodyPtr& robot_in);
    void calcManipulabilityJointLimit(fikPtr& fik_in, hrp::BodyPtr& robot_in, hrp::Vector3 target_v_old[], WBMSPose3D& rf_ref, WBMSPose3D& lf_ref, WBMSPose3D& rh_ref, WBMSPose3D& lh_ref);
    void calcManipulabilityJointLimitForWBMS(fikPtr& fik_in, hrp::BodyPtr& robot_in);
    void solveFullbodyIK(fikPtr& fik_in, hrp::BodyPtr& robot_in, const WBMSPose3D& com_ref, const WBMSPose3D& rf_ref, const WBMSPose3D& lf_ref, const WBMSPose3D& rh_ref, const WBMSPose3D& lh_ref, const WBMSPose3D& head_ref, const std::string& debug_prefix="");
    void processTransition();
    void preProcessForWholeBodyMasterSlave(fikPtr& fik_in, hrp::BodyPtr& robot_in);
    void processWholeBodyMasterSlave(fikPtr& fik_in, hrp::BodyPtr& robot_in, const HumanPose& pose_ref);
    void processMomentumCompensation(fikPtr& fik_in, hrp::BodyPtr& robot_in, hrp::BodyPtr& robot_normal_in, const HumanPose& pose_ref);
    void processHOFFARBIBFilter(hrp::BodyPtr& robot_in, hrp::BodyPtr& robot_out);
    bool isOptionalDataContact (const std::string& ee_name) { return (std::fabs(m_optionalData.data[contact_states_index_map[ee_name]]-1.0)<0.1)?true:false; };
};






extern "C"
{
    void WholeBodyMasterSlaveInit(RTC::Manager* manager);
};

#endif // WholeBodyMasterSlave_H
