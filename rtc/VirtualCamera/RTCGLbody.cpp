#include <rtm/DataFlowComponentBase.h>
#include "RTCGLbody.h"
#include "IrrModel.h"

RTCGLbody::RTCGLbody(GLbody *i_body, RTC::DataFlowComponentBase *comp) : 
    m_body(i_body),
    m_qIn("q", m_q),
    m_posIn("pos", m_pos),
    m_rpyIn("rpy", m_rpy),
    m_poseBaseIn("poseBase", m_poseBase)
{
    if (m_body->numJoints() > 0) comp->addInPort("q", m_qIn);
    if (m_body->rootLink()->jointType() == GLlink::FREE_JOINT){
        comp->addInPort("pos", m_posIn);
        comp->addInPort("rpy", m_rpyIn);
        comp->addInPort("poseBase", m_poseBaseIn);
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
        m_body->setOrientation(m_rpy.data.r,
                               m_rpy.data.p,
                               m_rpy.data.y);
    }
    if (m_poseBaseIn.isNew()){
        while (m_poseBaseIn.isNew()) m_poseBaseIn.read();
        m_body->setPosition(m_poseBase.data.position.x,
                            m_poseBase.data.position.y,
                            m_poseBase.data.position.z);
        m_body->setOrientation(m_poseBase.data.orientation.r,
                               m_poseBase.data.orientation.p,
                               m_poseBase.data.orientation.y);
    }
}
