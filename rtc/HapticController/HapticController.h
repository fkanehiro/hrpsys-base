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
//#include "../Stabilizer/StabilizerService_impl.h"
//#include "../AutoBalancer/AutoBalancerService_impl.h"
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
        void resetOdom();
        bool setParams(const OpenHRP::HapticControllerService::HapticControllerParam& i_param);
        bool getParams(OpenHRP::HapticControllerService::HapticControllerParam& i_param);
        typedef boost::shared_ptr<RTC::InPort   <RTC::TimedPose3D>      > ITP3_Ptr;
        typedef boost::shared_ptr<RTC::InPort   <RTC::TimedDoubleSeq>   > ITDS_Ptr;
        typedef boost::shared_ptr<RTC::OutPort  <RTC::TimedPose3D>      > OTP3_Ptr;
        typedef boost::shared_ptr<RTC::OutPort  <RTC::TimedDoubleSeq>   > OTDS_Ptr;

    protected:
        RTC::TimedDoubleSeq m_qRef;
        RTC::InPort<RTC::TimedDoubleSeq> m_qRefIn;
        RTC::TimedDoubleSeq m_qAct;
        RTC::InPort<RTC::TimedDoubleSeq> m_qActIn;
        RTC::TimedDoubleSeq m_dqAct;
        RTC::InPort<RTC::TimedDoubleSeq> m_dqActIn;
        std::map<std::string, RTC::TimedDoubleSeq> m_slaveEEWrenches;
        std::map<std::string, ITDS_Ptr> m_slaveEEWrenchesIn;
        RTC::TimedDoubleSeq m_tau;
        RTC::OutPort<RTC::TimedDoubleSeq> m_tauOut;
        RTC::TimedPose3D m_teleopOdom;
        RTC::OutPort<RTC::TimedPose3D> m_teleopOdomOut;
        RTC::OutPort<RTC::TimedDoubleSeq> m_qOut;
        std::map<std::string, RTC::TimedPose3D> m_masterTgtPoses;
        std::map<std::string, OTP3_Ptr> m_masterTgtPosesOut;
        // sub usage
        std::map<std::string, RTC::TimedDoubleSeq> m_masterEEWrenches;
        std::map<std::string, OTDS_Ptr> m_masterEEWrenchesOut;
        std::map<std::string, RTC::TimedPose3D> m_slaveTgtPoses;
        std::map<std::string, ITP3_Ptr> m_slaveTgtPosesIn;
        RTC::TimedDoubleSeq m_debugData;
        RTC::OutPort<RTC::TimedDoubleSeq> m_debugDataOut;
        RTC::CorbaPort m_HapticControllerServicePort;
        HapticControllerService_impl m_service0;

        RTC::Time m_delayCheckPacket;
        RTC::InPort<RTC::Time> m_delayCheckPacketInboundIn;
        RTC::OutPort<RTC::Time> m_delayCheckPacketOutboundOut;

    private:
        double m_dt;
        unsigned int loop;
        unsigned int m_debugLevel;
        hrp::BodyPtr m_robot, m_robot_odom;

        double output_ratio, q_ref_output_ratio, baselink_h_from_floor, default_baselink_h_from_floor;
        interpolator *t_ip, *q_ref_ip, *baselink_h_ip;

        hrp::InvDynStateBuffer idsb;
        BiquadIIRFilterVec dqAct_filter;
        hrp::dvector dqAct_filtered;
        std::map<std::string, IKConstraint> ee_ikc_map;
        std::map<std::string, BiquadIIRFilterVec> ee_vel_filter;
        std::map<std::string, BiquadIIRFilterVec> wrench_lpf_for_hpf, wrench_lpf;
        std::map<std::string, hrp::Pose3> master_ee_pose, master_ee_pose_old, slave_ee_pose, slave_ee_pose_old;
        std::map<std::string, hrp::dvector6> master_ee_vel, master_ee_vel_filtered, slave_ee_vel; // = twist
        std::map<std::string, hrp::JointPath> jpath_ee;
        std::map<std::string, hrp::dmatrix> J_ee;
        std::map<std::string, bool> is_contact_to_floor;
        std::map<std::string, double> foot_h_from_floor;
        bool resetOdom_request;

        double current_adjust_floor_h;

        ControlMode mode;

        std::vector<std::string> ee_names, tgt_names;
        class HCParams {
            public:
                double baselink_height_from_floor;
                double default_baselink_h_from_floor;
                double dqAct_filter_cutoff_hz;
                double ee_vel_filter_cutoff_hz;
                double ex_gravity_compensation_ratio_lower;
                double ex_gravity_compensation_ratio_upper;
                double foot_min_distance;
                double force_feedback_ratio;
                double gravity_compensation_ratio;
                double q_friction_coeff;
                double q_ref_max_torque_ratio;
                double torque_feedback_ratio;
                double wrench_hpf_cutoff_hz;
                double wrench_lpf_cutoff_hz;
                double wrench_hpf_gain;
                double wrench_lpf_gain;
                hrp::Vector2 ee_pos_rot_friction_coeff;
                hrp::Vector2 floor_pd_gain;
                hrp::Vector2 foot_horizontal_pd_gain;
                hrp::Vector2 force_feedback_limit_ft;
                hrp::Vector2 q_ref_pd_gain;
                std::map<std::string, hrp::dvector6> ex_ee_ref_wrench;
            HCParams(){
                baselink_height_from_floor          = 1.5;// will be overwrited
                dqAct_filter_cutoff_hz              = 500;// 10以下で確実に位相遅れによる振動
                ee_vel_filter_cutoff_hz             = 500;// 10以下で確実に位相遅れによる振動
                ex_gravity_compensation_ratio_lower = 1.0;
                ex_gravity_compensation_ratio_upper = 0.9;
                foot_min_distance                   = 0.25;
                force_feedback_ratio                = 0.1;
                gravity_compensation_ratio          = 1.0;
                q_friction_coeff                    = 0;
                q_ref_max_torque_ratio              = 0.01;
                torque_feedback_ratio               = 0.01;
                wrench_hpf_cutoff_hz                = 20;
                wrench_lpf_cutoff_hz                = 0.3;
                wrench_hpf_gain                     = 0.1;
                wrench_lpf_gain                     = 0;
                ee_pos_rot_friction_coeff           << 10, 0.1;
                floor_pd_gain                       << 10000, 500;
                foot_horizontal_pd_gain             << 300, 30;
                force_feedback_limit_ft             << 100, 10;
                q_ref_pd_gain                       << 50, 0;
                ex_ee_ref_wrench["rleg"]            = hrp::dvector6::Zero();
                ex_ee_ref_wrench["lleg"]            = hrp::dvector6::Zero();
                ex_ee_ref_wrench["rarm"]            = hrp::dvector6::Zero();
                ex_ee_ref_wrench["larm"]            = hrp::dvector6::Zero();
                CheckSafeLimit();
            }
            void CheckSafeLimit(){
                LIMIT_MINMAX(baselink_height_from_floor          , 0.5, 3);
                LIMIT_MINMAX(dqAct_filter_cutoff_hz              , 0, 500);
                LIMIT_MINMAX(ee_vel_filter_cutoff_hz             , 0, 500);
                LIMIT_MINMAX(ex_gravity_compensation_ratio_lower , -1, 2);
                LIMIT_MINMAX(ex_gravity_compensation_ratio_upper , -1, 2);
                LIMIT_MINMAX(foot_min_distance                   , 0, 1);
                LIMIT_MINMAX(force_feedback_ratio                , 0, 2);
                LIMIT_MINMAX(gravity_compensation_ratio          , 0, 2);
                LIMIT_MINMAX(q_friction_coeff                    , 0, 0.1);
                LIMIT_MINMAX(q_ref_max_torque_ratio              , 0, 2);
                LIMIT_MINMAX(torque_feedback_ratio               , 0, 1);
                LIMIT_MINMAX(wrench_hpf_cutoff_hz                , 0, 500);
                LIMIT_MINMAX(wrench_lpf_cutoff_hz                , 0, 500);
                LIMIT_MINMAX(wrench_hpf_gain                     , 0, 2);
                LIMIT_MINMAX(wrench_lpf_gain                     , 0, 2);
            }
        } hcp;

        RTC::ReturnCode_t setupEEIKConstraintFromConf(std::map<std::string, IKConstraint>& _ee_ikc_map, hrp::BodyPtr _robot, RTC::Properties& _prop);
        void calcCurrentState();
        void calcTorque();
        void calcOdometry();
        void processTransition();
};


extern "C" {    void HapticControllerInit(RTC::Manager* manager);   };

#endif
