#include <iostream>
#include <hrpModel/Sensor.h>
#include <hrpModel/Link.h>
#include "BodyRTC.h"

using namespace hrp;
using namespace RTC;

// Module specification
// <rtc-template block="module_spec">
const char* BodyRTC::bodyrtc_spec[] =
{
    "implementation_id", "BodyRTC",
    "type_name",         "BodyRTC",
    "description",       "BodyRTC component",
    "version",           "0.1",
    "vendor",            "AIST",
    "category",          "Generic",
    "activity_type",     "DataFlowComponent",
    "max_instance",      "10",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables

    ""
};

BodyRTC::BodyRTC(RTC::Manager* manager)
    : Body(),
      DataFlowComponentBase(manager),
      m_tauIn("tau", m_tau),
      m_qRefIn("qRef", m_qRef),
      m_dqRefIn("dqRef", m_dqRef),
      m_ddqRefIn("ddqRef", m_ddqRef),
      m_qOut("q", m_q),
      dummy(0)
{
    //std::cout << "constructor of BodyRTC"  << std::endl;
}

BodyRTC::BodyRTC(const BodyRTC& i_body) 
    : Body(i_body),
      DataFlowComponentBase(&RTC::Manager::instance()),
      m_tauIn("tau", m_tau),
      m_qRefIn("qRef", m_qRef),
      m_dqRefIn("dqRef", m_dqRef),
      m_ddqRefIn("ddqRef", m_ddqRef),
      m_qOut("q", m_q),
      dummy(0)
{
    std::cout << "BodyRTC::BodyRTC(const BodyRTC&) is called" << std::endl;
}

BodyRTC::~BodyRTC(void)
{
    //std::cout << "destructor of BodyRTC"  << std::endl;
}

void BodyRTC::createDataPorts()
{
    if (numJoints()){
        bool isHighGainMode = false;
        for (int i=0;i<numJoints(); i++){
            hrp::Link *j = joint(i);
            if (j && j->isHighGainMode){
                isHighGainMode = true;
                break;
            }
        }
        if (isHighGainMode){
            std::cout << name() << ": high-gain mode" << std::endl;
            m_qRef.data.length(numJoints());
            m_dqRef.data.length(numJoints());
            m_ddqRef.data.length(numJoints());
            for (int i=0; i<numJoints(); i++){
                m_qRef.data[i] = m_dqRef.data[i] = m_ddqRef.data[i] = 0;
            }
            addInPort("qRef", m_qRefIn); 
            addInPort("dqRef", m_dqRefIn); 
            addInPort("ddqRef", m_ddqRefIn); 
        }else{
            std::cout << name() << ": toque-given mode" << std::endl;
            m_tau.data.length(numJoints());
            for (int i=0; i<numJoints(); i++) m_tau.data[i] = 0;
            addInPort("tau", m_tauIn);
        }

        m_q.data.length(numJoints());
        addOutPort("q", m_qOut);
    }

    int ngyro = numSensors(Sensor::RATE_GYRO);
    //std::cout << "the number of gyros = " << ngyro << std::endl;
    m_rate.resize(ngyro);
    m_rateOut.resize(ngyro);
    for (unsigned int i=0; i<m_rate.size(); i++){
        Sensor *s = sensor(Sensor::RATE_GYRO, i);
        //std::cout << s->name << std::endl;
        m_rateOut[i] = new OutPort<TimedAngularVelocity3D>(s->name.c_str(), m_rate[i]);
        addOutPort(s->name.c_str(), *m_rateOut[i]);
    }

    int nacc = numSensors(Sensor::ACCELERATION);
    //std::cout << "the number of accelerometers = " << nacc << std::endl;
    m_acc.resize(nacc);
    m_accOut.resize(nacc);
    for (unsigned int i=0; i<m_acc.size(); i++){
        Sensor *s = sensor(Sensor::ACCELERATION, i);
        //std::cout << s->name << std::endl;
        m_accOut[i] = new OutPort<TimedAcceleration3D>(s->name.c_str(), m_acc[i]);
        addOutPort(s->name.c_str(), *m_accOut[i]);
    }

    int nforce = numSensors(Sensor::FORCE);
    //std::cout << "the number of force sensors = " << nforce << std::endl;
    m_force.resize(nforce);
    m_forceOut.resize(nforce);
    for (unsigned int i=0; i<m_force.size(); i++){
        Sensor *s = sensor(Sensor::FORCE, i);
        //std::cout << s->name << std::endl;
        m_forceOut[i] = new OutPort<TimedDoubleSeq>(s->name.c_str(), m_force[i]);
        m_force[i].data.length(6);
        addOutPort(s->name.c_str(), *m_forceOut[i]);
    }
}

void BodyRTC::writeDataPorts()
{
    if (numJoints()){
        for (int i=0; i<numJoints(); i++){
            Link *l = joint(i);
            if (l){
                m_q.data[l->jointId] = l->q;
            }
        }
        m_qOut.write();
    }

    Link *root = rootLink();

    int n = numSensors(Sensor::FORCE);
    for(int id = 0; id < n; ++id){
        ForceSensor* s = sensor<ForceSensor>(id);
        setVector3(s->f,   m_force[id].data, 0);
        setVector3(s->tau, m_force[id].data, 3);
    }
    for (unsigned int i=0; i<m_forceOut.size(); i++){
        m_forceOut[i]->write();
    }

    n = numSensors(Sensor::RATE_GYRO);
    for(int id=0; id < n; ++id){
        RateGyroSensor* s = sensor<RateGyroSensor>(id);
        m_rate[id].data.avx = s->w[0];
        m_rate[id].data.avy = s->w[1];
        m_rate[id].data.avz = s->w[2];
    }
    for (unsigned int i=0; i<m_rateOut.size(); i++){
        m_rateOut[i]->write();
    }
    
    n = numSensors(Sensor::ACCELERATION);
    for(int id=0; id < n; ++id){
        AccelSensor* s = sensor<AccelSensor>(id);
        m_acc[id].data.ax = s->dv[0];
        m_acc[id].data.ay = s->dv[1];
        m_acc[id].data.az = s->dv[2];
    }		
    for (unsigned int i=0; i<m_accOut.size(); i++){
        m_accOut[i]->write();
    }
}

void BodyRTC::readDataPorts()
{
    if(m_tauIn.isNew()){
        do {
            m_tauIn.read();
        }while (m_tauIn.isNew());
        for (int i=0; i<numJoints(); i++){
            Link *l = joint(i);
            if (l){
                l->u = m_tau.data[i];
            }
        }
    }
    if (m_qRefIn.isNew()){
        do {
            m_qRefIn.read();
        }while(m_qRefIn.isNew());
        for (int i=0; i<numJoints(); i++){
            Link *l = joint(i);
            if (l){
                l->q = m_qRef.data[i];
            }
        }
    }
    if (m_dqRefIn.isNew()){
        do {
            m_dqRefIn.read();
        }while(m_dqRefIn.isNew());
        for (int i=0; i<numJoints(); i++){
            Link *l = joint(i);
            if (l){
                l->dq = m_dqRef.data[i];
            }
        }
    }
    if (m_ddqRefIn.isNew()){
        do {
            m_ddqRefIn.read();
        }while(m_ddqRefIn.isNew());
        for (int i=0; i<numJoints(); i++){
            Link *l = joint(i);
            if (l){
                l->ddq = m_ddqRef.data[i];
            }
        }
    }
}

template <class _Delete>
void DummyDelete(RTC::RTObject_impl* rtc)
{
    // BodyRTC will be released when BodyPtr is released
}

void BodyRTC::moduleInit(RTC::Manager* manager)
{
    coil::Properties profile(bodyrtc_spec);
    manager->registerFactory(profile,
                             RTC::Create<BodyRTC>,
                             DummyDelete<BodyRTC>
                             //RTC::Delete<BodyRTC>
        );
}
