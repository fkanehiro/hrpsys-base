#include <hrpModel/Link.h>
#include <hrpModel/Sensor.h>
#include "RTCBody.h"

using namespace hrp;

RTCBody::RTCBody() :
    m_tauIn("tau", m_tau),
    m_qCmdIn("qCmd", m_qCmd),
    m_dqCmdIn("dqCmd", m_dqCmd),
    m_ddqCmdIn("ddqCmd", m_ddqCmd),
    m_posOut("pos", m_pos),
    m_rpyOut("rpy", m_rpy),
    m_qOut("q", m_q),
    m_highgain(false)
{
}

RTCBody::~RTCBody()
{
}

void RTCBody::createPorts(RTC::DataFlowComponentBase *comp)
{
    // input ports
    for (unsigned int i=0; i<numJoints(); i++){
        Link *l = joint(i);
        if (l){
            if (l->isHighGainMode){
                m_highgain = true;
                break;
            }
        }
    }

    if (numJoints()){
        if (m_highgain){
            comp->addInPort("qCmd", m_qCmdIn);
            comp->addInPort("dqCmd", m_dqCmdIn);
            comp->addInPort("ddqCmd", m_ddqCmdIn);
        }else{
            comp->addInPort("tau", m_tauIn);
        }
    }

    // output ports
    m_q.data.length(numJoints());
    if (numJoints() > 0) comp->addOutPort("q", m_qOut);
    if (rootLink()->jointType == Link::FREE_JOINT){
        comp->addOutPort("pos", m_posOut);
        comp->addOutPort("rpy", m_rpyOut);
    }

    int ngyro = numSensors(Sensor::RATE_GYRO);
    m_rate.resize(ngyro);
    m_rateOut.resize(ngyro);
    for (unsigned int i=0; i<m_rate.size(); i++){
        Sensor *s = sensor(Sensor::RATE_GYRO, i);
        m_rateOut[i] = new RTC::OutPort<RTC::TimedAngularVelocity3D>(s->name.c_str(), m_rate[i]);
        comp->addOutPort(s->name.c_str(), *m_rateOut[i]);
    }
    
    int nacc = numSensors(Sensor::ACCELERATION);
    m_acc.resize(nacc);
    m_accOut.resize(nacc);
    for (unsigned int i=0; i<m_acc.size(); i++){
        Sensor *s = sensor(Sensor::ACCELERATION, i);
        m_accOut[i] = new RTC::OutPort<RTC::TimedAcceleration3D>(s->name.c_str(), m_acc[i]);
        comp->addOutPort(s->name.c_str(), *m_accOut[i]);
    }
    
    int nforce = numSensors(Sensor::FORCE);
    m_force.resize(nforce);
    m_forceOut.resize(nforce);
    for (unsigned int i=0; i<m_force.size(); i++){
        Sensor *s = sensor(Sensor::FORCE, i);
        m_forceOut[i] = new RTC::OutPort<RTC::TimedDoubleSeq>(s->name.c_str(), 
                                                              m_force[i]);
        m_force[i].data.length(6);
        comp->addOutPort(s->name.c_str(), *m_forceOut[i]);
    }
}

void RTCBody::input()
{
    if (m_highgain){
        if (m_qCmdIn.isNew()){
            do {
                m_qCmdIn.read();
            }while (m_qCmdIn.isNew());
            for (unsigned int i=0; i<numJoints(); i++){
                Link *l = joint(i);
                if (l){
                    l->q = m_qCmd.data[l->jointId];
                }
            }
        }
        if (m_dqCmdIn.isNew()){
            do {
                m_dqCmdIn.read();
            }while (m_dqCmdIn.isNew());
            for (unsigned int i=0; i<numJoints(); i++){
                Link *l = joint(i);
                if (l){
                    l->dq = m_dqCmd.data[l->jointId];
                }
            }
        }
        if (m_ddqCmdIn.isNew()){
            do {
                m_ddqCmdIn.read();
            }while (m_ddqCmdIn.isNew());
            for (unsigned int i=0; i<numJoints(); i++){
                Link *l = joint(i);
                if (l){
                    l->ddq = m_ddqCmd.data[l->jointId];
                }
            }
        }
    }else{
        if (m_tauIn.isNew()){
            do {
                m_tauIn.read();
            }while (m_tauIn.isNew());
            for (unsigned int i=0; i<numJoints(); i++){
                Link *l = joint(i);
                if (l){
                    l->u = m_tau.data[l->jointId];
                }
            }
        }
    }
}

void RTCBody::output(OpenHRP::RobotState& state)
{
    if (numJoints() > 0){
        for (unsigned int i=0; i<numJoints(); i++){
            Link *l = joint(i);
            if (l){
                m_q.data[l->jointId] = l->q;
            }
        }
        state.q = m_q.data;
        m_qOut.write();
    }

    Link *root = rootLink();

    m_pos.data.x = root->p[0];
    m_pos.data.y = root->p[1];
    m_pos.data.z = root->p[2];
    state.basePose.position = m_pos.data;
    
    Vector3 rpy(rpyFromRot(root->attitude()));
    m_rpy.data.r = rpy[0];
    m_rpy.data.p = rpy[1];
    m_rpy.data.y = rpy[2];
    state.basePose.orientation = m_rpy.data;

    if (root->jointType == Link::FREE_JOINT){
        m_posOut.write();
        m_rpyOut.write();
    }

    int n = numSensors(Sensor::FORCE);
    for(int id = 0; id < n; ++id){
        ForceSensor* s = sensor<ForceSensor>(id);
        setVector3(s->f,   m_force[id].data, 0);
        setVector3(s->tau, m_force[id].data, 3);
    }

    n = numSensors(Sensor::RATE_GYRO);
    for(int id=0; id < n; ++id){
        RateGyroSensor* s = sensor<RateGyroSensor>(id);
        m_rate[id].data.avx = s->w[0];
        m_rate[id].data.avy = s->w[1];
        m_rate[id].data.avz = s->w[2];
    }
    
    n = numSensors(Sensor::ACCELERATION);
    for(int id=0; id < n; ++id){
        AccelSensor* s = sensor<AccelSensor>(id);
        m_acc[id].data.ax = s->dv[0];
        m_acc[id].data.ay = s->dv[1];
        m_acc[id].data.az = s->dv[2];
    }		
}

