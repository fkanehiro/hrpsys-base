#include <rtm/DataFlowComponentBase.h>
#include "RTCGLbody.h"
#if 0
#include "IrrModel.h"
#else
#include "util/GLbody.h"
#include "util/GLlink.h"
#endif

RTCGLbody::RTCGLbody(GLbody *i_body, RTC::DataFlowComponentBase *comp) : 
    m_body(i_body),
    m_qIn("q", m_q),
    m_posIn("pos", m_pos),
    m_rpyIn("rpy", m_rpy)
{
    if (m_body->numJoints() > 0) comp->addInPort("q", m_qIn);
    if (m_body->rootLink()->jointType == hrp::Link::FREE_JOINT){
        comp->addInPort("pos", m_posIn);
        comp->addInPort("rpy", m_rpyIn);
    }
}

void RTCGLbody::input()
{
    if (m_qIn.isNew()){
        while (m_qIn.isNew()) m_qIn.read();
        m_body->setPosture(m_q.data.get_buffer());
    }
    if (m_posIn.isNew()){
        while (m_posIn.isNew()) m_posIn.read();
        m_body->setPosition(m_pos.data.x,
                            m_pos.data.y,
                            m_pos.data.z);
    }
    if (m_rpyIn.isNew()){
        while (m_rpyIn.isNew()) m_rpyIn.read();
        m_body->setRotation(m_rpy.data.r,
                            m_rpy.data.p,
                            m_rpy.data.y);
    }
}
