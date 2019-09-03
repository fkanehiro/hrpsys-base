#ifndef HAPTICCONTROLLER_H
#define HAPTICCONTROLLER_H

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

#include "HapticControllerService_impl.h"
#include "../Stabilizer/StabilizerService_impl.h"
#include "../AutoBalancer/AutoBalancerService_impl.h"
//#include "wbms_core.h"
#include "../AutoBalancer/FullbodyInverseKinematicsSolver.h"

#include "../SequencePlayer/interpolator.h"
#include "../ImpedanceController/JointPathEx.h"

enum mode_enum{ MODE_IDLE, MODE_SYNC_TO_HC, MODE_HC, MODE_PAUSE, MODE_SYNC_TO_IDLE};

class ControlMode{
    private:
        mode_enum current, previous, next;
    public:
        ControlMode(){ current = previous = next = MODE_IDLE;}
        ~ControlMode(){}
        bool setNextMode(const mode_enum _request){
            switch(_request){
                case MODE_SYNC_TO_HC:
                    if(current == MODE_IDLE)                                    { next = _request; return true; }else{ return false; }
                case MODE_HC:
                    if(current == MODE_SYNC_TO_HC || current == MODE_PAUSE )  { next = _request; return true; }else{ return false; }
                case MODE_PAUSE:
                    if(current == MODE_HC)                                    { next = _request; return true; }else{ return false; }
                case MODE_SYNC_TO_IDLE:
                    if(current == MODE_HC || current == MODE_PAUSE )          { next = _request; return true; }else{ return false; }
                case MODE_IDLE:
                    if(current == MODE_SYNC_TO_IDLE )                           { next = _request; return true; }else{ return false; }
                default:
                    return false;
            }
        }
        void update(){ previous = current; current = next; }
        mode_enum now(){ return current; }
        mode_enum pre(){ return previous; }
        bool isRunning(){ return (current==MODE_SYNC_TO_HC) || (current==MODE_HC) || (current==MODE_PAUSE) || (current==MODE_SYNC_TO_IDLE) ;}
        bool isInitialize(){ return (previous==MODE_IDLE) && (current==MODE_SYNC_TO_HC) ;}
};

class HapticController : public RTC::DataFlowComponentBase{
    public:
        HapticController(RTC::Manager* manager);
        virtual ~HapticController();
        virtual RTC::ReturnCode_t onInitialize();
        virtual RTC::ReturnCode_t onFinalize();
        virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);
        virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);
        virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);
        bool startHapticController();
        bool stopHapticController();
        bool pauseHapticController();
        bool resumeHapticController();
        bool setParams(const OpenHRP::HapticControllerService::HapticControllerParam& i_param);
        bool getParams(OpenHRP::HapticControllerService::HapticControllerParam& i_param);

    protected:

        RTC::TimedDoubleSeq m_qRef;
        RTC::InPort<RTC::TimedDoubleSeq> m_qRefIn;
        RTC::TimedDoubleSeq m_qAct;
        RTC::InPort<RTC::TimedDoubleSeq> m_qActIn;
        RTC::TimedDoubleSeq m_dqAct;
        RTC::InPort<RTC::TimedDoubleSeq> m_dqActIn;
        std::map<std::string, RTC::TimedDoubleSeq> m_feedbackWrenches;
        typedef boost::shared_ptr<RTC::InPort<RTC::TimedDoubleSeq> > ITDS_Ptr;
        std::map<std::string, ITDS_Ptr> m_feedbackWrenchesIn;

        RTC::TimedDoubleSeq m_tau;
        RTC::OutPort<RTC::TimedDoubleSeq> m_tauOut;
        RTC::TimedPose3D m_teleopOdom;
        RTC::OutPort<RTC::TimedPose3D> m_teleopOdomOut;
        RTC::OutPort<RTC::TimedDoubleSeq> m_qOut;
        std::map<std::string, RTC::TimedPose3D> m_masterTgtPoses;
        typedef boost::shared_ptr<RTC::OutPort<RTC::TimedPose3D> > OTP3_Ptr;
        std::map<std::string, OTP3_Ptr> m_eePosesOut;
        RTC::TimedDoubleSeq m_debugData;
        RTC::OutPort<RTC::TimedDoubleSeq> m_debugDataOut;

//        RTC::TimedDoubleSeq m_dq_filtered_dbg;
//        RTC::OutPort<RTC::TimedDoubleSeq> m_dq_filtered_dbgOut;
//        std::map<std::string, RTC::TimedDoubleSeq> m_feedbackWrenches_filtered_dbg;
//        RTC::OutPort<RTC::TimedDoubleSeq> m_dq_filtered_dbgOut;



        RTC::CorbaPort m_HapticControllerServicePort;

        hrp::BodyPtr m_robot;

        HapticControllerService_impl m_service0;

    private:
        double m_dt;
        unsigned int loop;
        unsigned int m_debugLevel;

        class HCParams {
            public:
                double gravity_compensation_ratio;
                double force_feedback_ratio;
                double dqAct_filter_cutoff_hz;
                double ee_vel_filter_cutoff_hz;
                double wrench_filter_cutoff_hz;
                double q_friction_coeff;
                hrp::Vector2 ee_friction_coeff;
                double wrench_hpf_cutoff_hz;
                double wrench_lpf_cutoff_hz;
                double wrench_hpf_gain;
                double wrench_lpf_gain;
            HCParams(){
                gravity_compensation_ratio  = 0.9;
                force_feedback_ratio        = 0.2;
                dqAct_filter_cutoff_hz      = 100;// 10以下で確実に位相遅れによる振動
                ee_vel_filter_cutoff_hz     = 100;// 10以下で確実に位相遅れによる振動
                wrench_filter_cutoff_hz     = 100;
                q_friction_coeff            = 1.0;
                ee_friction_coeff           << 1, 0.1;
                wrench_hpf_cutoff_hz        = 20;
                wrench_lpf_cutoff_hz        = 0.3;
                wrench_hpf_gain             = 1;
                wrench_lpf_gain             = 0.2;
            }
        } hcp;


        double output_ratio;
        interpolator *t_ip,*q_ip;
        hrp::dvector avg_q_vel, avg_q_acc;

        hrp::InvDynStateBuffer idsb;
        BiquadIIRFilterVec2 dqAct_filter;
        hrp::dvector dqAct_filtered;
        std::map<std::string, BiquadIIRFilterVec2> ee_vel_filter;
        std::map<std::string, hrp::dvector6> ee_vel_filtered;
        std::map<std::string, BiquadIIRFilterVec2> wrench_filter;
        std::map<std::string, hrp::dvector6> wrench_filtered;

        std::map<std::string, BiquadIIRFilterVec2> wrench_lpf_for_hpf;
        std::map<std::string, BiquadIIRFilterVec2> wrench_lpf;
        std::map<std::string, hrp::dvector6> wrench_shaped;
        std::map<std::string, hrp::dvector6> wrench_used;

        std::map<std::string, IKConstraint> ee_ikc_map;
        ControlMode mode;

        std::vector<std::string> ee_names;
        std::vector<std::string> tgt_names;

        RTC::ReturnCode_t setupEEIKConstraintFromConf(std::map<std::string, IKConstraint>& _ee_ikc_map, hrp::BodyPtr _robot, RTC::Properties& _prop);
        void calcTorque();
        void processTransition();
//        void processHapticController(const HumanPose& pose_ref);
};


extern "C" {    void HapticControllerInit(RTC::Manager* manager);   };

#endif
