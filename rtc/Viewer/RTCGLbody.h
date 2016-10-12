#ifndef __RTCGLBODY_H__
#define __RTCGLBODY_H__

#include <rtm/idl/BasicDataType.hh>
#include <rtm/idl/ExtendedDataTypes.hh>
#include <rtm/DataInPort.h>
#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/idl/ExtendedDataTypesSkel.h>

class GLbody;
namespace RTC
{
    class DataFlowComponentBase;
};

class RTCGLbody
{
public:
    RTCGLbody(GLbody *i_body, RTC::DataFlowComponentBase *comp);
    void input();
    GLbody *body() { return m_body; }
private:
    GLbody *m_body;
    RTC::TimedDoubleSeq m_q;
    RTC::TimedPoint3D m_pos;
    RTC::TimedOrientation3D m_rpy;

    RTC::InPort<RTC::TimedDoubleSeq> m_qIn;
    RTC::InPort<RTC::TimedPoint3D> m_posIn;
    RTC::InPort<RTC::TimedOrientation3D> m_rpyIn;
};
#endif
