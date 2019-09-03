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
#include <rtm/CorbaNaming.h>
#include <hrpModel/ModelLoaderUtil.h>
#include <hrpModel/Sensor.h>
#include <time.h>

#include "WholeBodyMasterSlaveService_impl.h"
#include "../Stabilizer/StabilizerService_impl.h"
#include "../AutoBalancer/AutoBalancerService_impl.h"
#include "wbms_core.h"

//#define USE_DEBUG_PORT

enum mode_enum{ MODE_IDLE, MODE_SYNC_TO_HC, MODE_WBMS, MODE_PAUSE, MODE_SYNC_TO_IDLE};

class ControlMode{
    private:
        mode_enum current, previous, next;
    public:
        ControlMode(){ current = previous = next = MODE_IDLE;}
        ~ControlMode(){}
        bool setNextMode(const mode_enum _request){
            switch(_request){
                case MODE_SYNC_TO_HC:
                    if(current == MODE_IDLE){ next = MODE_SYNC_TO_HC; return true; }else{ return false; }
                case MODE_WBMS:
                    if(current == MODE_SYNC_TO_HC || current == MODE_PAUSE ){ next = MODE_WBMS; return true; }else{ return false; }
                case MODE_PAUSE:
                    if(current == MODE_WBMS){ next = MODE_PAUSE; return true; }else{ return false; }
                case MODE_SYNC_TO_IDLE:
                    if(current == MODE_WBMS || current == MODE_PAUSE ){ next = MODE_SYNC_TO_IDLE; return true; }else{ return false; }
                case MODE_IDLE:
                    if(current == MODE_SYNC_TO_IDLE ){ next = MODE_IDLE; return true; }else{ return false; }
                default:
                    return false;
            }
        }
        void update(){ previous = current; current = next; }
        mode_enum now(){ return current; }
        mode_enum pre(){ return previous; }
        bool isRunning(){ return (current==MODE_SYNC_TO_HC) || (current==MODE_WBMS) || (current==MODE_PAUSE) || (current==MODE_SYNC_TO_IDLE) ;}
        bool isInitialize(){ return (previous==MODE_IDLE) && (current==MODE_SYNC_TO_HC) ;}
};

namespace hrp{
    inline std::vector<std::string> to_string_vector (const OpenHRP::WholeBodyMasterSlaveService::StrSequence& in) {
        std::vector<std::string> ret(in.length()); for(int i=0; i<in.length(); i++){ ret[i] = in[i]; } return ret;
    }
    inline OpenHRP::WholeBodyMasterSlaveService::StrSequence    to_StrSequence  (const std::vector<std::string>& in){
        OpenHRP::WholeBodyMasterSlaveService::StrSequence ret; ret.length(in.size()); for(int i=0; i<in.size(); i++){ ret[i] = in[i].c_str(); } return ret;
    }
}

class WholeBodyMasterSlave : public RTC::DataFlowComponentBase{
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
        bool setParams(const OpenHRP::WholeBodyMasterSlaveService::WholeBodyMasterSlaveParam& i_param);
        bool getParams(OpenHRP::WholeBodyMasterSlaveService::WholeBodyMasterSlaveParam& i_param);

    protected:
        RTC::TimedDoubleSeq m_qRef;
        RTC::InPort<RTC::TimedDoubleSeq> m_qRefIn;
        RTC::TimedDoubleSeq m_qAct;
        RTC::InPort<RTC::TimedDoubleSeq> m_qActIn;
        RTC::TimedPoint3D m_basePos;
        RTC::InPort<RTC::TimedPoint3D> m_basePosIn;
        RTC::TimedOrientation3D m_baseRpy;
        RTC::InPort<RTC::TimedOrientation3D> m_baseRpyIn;
        RTC::TimedPoint3D m_zmp;
        RTC::InPort<RTC::TimedPoint3D> m_zmpIn;
        RTC::TimedDoubleSeq m_optionalData;
        RTC::InPort<RTC::TimedDoubleSeq> m_optionalDataIn;

        std::map<std::string, RTC::TimedPose3D> m_masterTgtPoses;
        typedef boost::shared_ptr<RTC::InPort<RTC::TimedPose3D> > ITP3_Ptr;
        std::map<std::string, ITP3_Ptr> m_masterTgtPosesIn;

        std::map<std::string, RTC::TimedDoubleSeq> m_slaveEEWrenches;
        typedef boost::shared_ptr<RTC::OutPort<RTC::TimedDoubleSeq> > OTDS_Ptr;
        std::map<std::string, OTDS_Ptr> m_slaveEEWrenchesOut;

        std::map<std::string, RTC::TimedDoubleSeq> m_localEEWrenches;
        typedef boost::shared_ptr<RTC::InPort<RTC::TimedDoubleSeq> > ITDS_Ptr;
        std::map<std::string, ITDS_Ptr> m_localEEWrenchesIn;

        RTC::TimedDoubleSeq m_exData;
        RTC::InPort<RTC::TimedDoubleSeq> m_exDataIn;
        RTC::TimedStringSeq m_exDataIndex;
        RTC::InPort<RTC::TimedStringSeq> m_exDataIndexIn;

        RTC::TimedPoint3D m_actCP;
        RTC::InPort<RTC::TimedPoint3D> m_actCPIn;
        RTC::TimedPoint3D m_actZMP;
        RTC::InPort<RTC::TimedPoint3D> m_actZMPIn;

        RTC::OutPort<RTC::TimedDoubleSeq> m_qOut;
        RTC::OutPort<RTC::TimedPoint3D> m_zmpOut;
        RTC::OutPort<RTC::TimedPoint3D> m_basePosOut;
        RTC::OutPort<RTC::TimedOrientation3D> m_baseRpyOut;
        RTC::OutPort<RTC::TimedDoubleSeq> m_optionalDataOut;
        RTC::CorbaPort m_WholeBodyMasterSlaveServicePort;

        WholeBodyMasterSlaveService_impl m_service0;

        std::map<std::string, int> to_enum;

        RTC::CorbaPort m_AutoBalancerServicePort;
        RTC::CorbaPort m_StabilizerServicePort;
        RTC::CorbaConsumer<OpenHRP::AutoBalancerService> m_AutoBalancerServiceConsumer;
        RTC::CorbaConsumer<OpenHRP::StabilizerService> m_StabilizerServiceConsumer;


    private:
        double m_dt;
        unsigned int loop;
        unsigned int m_debugLevel;
        int optionalDataLength;
        bool is_legged_robot;
        hrp::BodyPtr m_robot_act; // actual
        hrp::BodyPtr m_robot_vsafe; // joint trajectory safe
        typedef boost::shared_ptr<FullbodyInverseKinematicsSolver> fikPtr;
        fikPtr fik;
        std::map<std::string, IKConstraint> ee_ikc_map; // e.g. feet hands head com
        std::map<std::string, size_t> contact_states_index_map;

        double output_ratio;
        interpolator *t_ip,*q_ip;
        hrp::dvector avg_q_vel, avg_q_acc;

        hrp::InvDynStateBuffer idsb;
    //    BiquadIIRFilterVec invdyn_zmp_filters;
        BiquadIIRFilterVec ref_zmp_filter;
        IIRFilter ref_ee_vel_v_filter;

        HumanPose raw_pose;

        boost::shared_ptr<WBMSCore> wbms;
        boost::shared_ptr<CapsuleCollisionChecker> sccp;

        hrp::Vector3 torso_rot_rmc;
        ControlMode mode;

        hrp::Vector3 rel_act_cp;
        hrp::Vector3 rel_act_zmp;
        int cp_flag;
        hrp::Vector3 lt,rt;
        enum cp_enum{ CP_IDLE, CP_LF, CP_RF, CP_STATIC};
        struct timespec startT, endT;
        std::string time_report_str;

        std::vector<std::string> ee_names;
        std::vector<std::string> tgt_names;

        RTC::ReturnCode_t setupEEIKConstraintFromConf(std::map<std::string, IKConstraint>& _ee_ikc_map, hrp::BodyPtr _robot, RTC::Properties& _prop);
        void solveFullbodyIK(const hrp::Pose3& com_ref, const hrp::Pose3& rf_ref, const hrp::Pose3& lf_ref, const hrp::Pose3& rh_ref, const hrp::Pose3& lh_ref, const hrp::Pose3& head_ref);
        void processTransition();
        void preProcessForWholeBodyMasterSlave();
        void processWholeBodyMasterSlave(const HumanPose& pose_ref);
        void smoothingJointAngles(hrp::BodyPtr _robot, hrp::BodyPtr _robot_safe);
        bool isOptionalDataContact (const std::string& ee_name) { return (std::fabs(m_optionalData.data[contact_states_index_map[ee_name]]-1.0)<0.1)?true:false; }
        void addTimeReport(const std::string& prefix){
            clock_gettime(CLOCK_REALTIME, &endT);
            std::stringstream ss;
            ss << prefix << "= " << std::fixed <<std::setprecision(2) << (double)(endT.tv_sec - startT.tv_sec + (endT.tv_nsec - startT.tv_nsec) * 1e-6) << " [ms] / ";
            time_report_str += ss.str();
            clock_gettime(CLOCK_REALTIME, &startT);
        }
};


extern "C"
{
    void WholeBodyMasterSlaveInit(RTC::Manager* manager);
};

#endif // WholeBodyMasterSlave_H
