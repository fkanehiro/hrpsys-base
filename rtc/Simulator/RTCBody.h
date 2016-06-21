#ifndef __RTCBODY_H__
#define __RTCBODY_H__

#include <rtm/idl/BasicDataType.hh>
#include <rtm/idl/ExtendedDataTypes.hh>
#include "hrpsys/idl/HRPDataTypes.hh"
#include <hrpModel/Body.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>
#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/idl/ExtendedDataTypesSkel.h>

class RTCBody : public hrp::Body
{
public:
    RTCBody();
    ~RTCBody();
    void createPorts(RTC::DataFlowComponentBase *comp);
    void input();
    void output(OpenHRP::RobotState& state);
private:
    RTC::TimedDoubleSeq m_tau;
    RTC::TimedDoubleSeq m_qCmd, m_dqCmd, m_ddqCmd;

    RTC::InPort<RTC::TimedDoubleSeq> m_tauIn;
    RTC::InPort<RTC::TimedDoubleSeq> m_qCmdIn;
    RTC::InPort<RTC::TimedDoubleSeq> m_dqCmdIn;
    RTC::InPort<RTC::TimedDoubleSeq> m_ddqCmdIn;
    //
    RTC::TimedPoint3D m_pos;
    RTC::TimedOrientation3D m_rpy;
    RTC::TimedDoubleSeq m_q;
    std::vector<RTC::TimedAcceleration3D> m_acc;
    std::vector<RTC::TimedAngularVelocity3D> m_rate;
    std::vector<RTC::TimedDoubleSeq> m_force;

    RTC::OutPort<RTC::TimedPoint3D> m_posOut;
    RTC::OutPort<RTC::TimedOrientation3D> m_rpyOut;
    RTC::OutPort<RTC::TimedDoubleSeq> m_qOut;
    std::vector<RTC::OutPort<RTC::TimedAcceleration3D> *> m_accOut;
    std::vector<RTC::OutPort<RTC::TimedAngularVelocity3D> *> m_rateOut;
    std::vector<RTC::OutPort<RTC::TimedDoubleSeq> *> m_forceOut;

    bool m_highgain;
};

typedef boost::shared_ptr<RTCBody> RTCBodyPtr;

#endif
