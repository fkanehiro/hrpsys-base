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
      m_dqOut("dq", m_dq),
      m_basePoseOut("basePose", m_basePose),
      dummy(0)
{
    //std::cout << "constructor of BodyRTC"  << std::endl;
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
        m_dq.data.length(numJoints());
        addOutPort("q", m_qOut);
        addOutPort("dq", m_dqOut);
        addOutPort("basePose", m_basePoseOut);
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

    int nrange = numSensors(Sensor::RANGE);
    m_range.resize(nrange);
    m_rangeOut.resize(nrange);
    for (size_t i=0; i<m_range.size(); i++){
        Sensor *s = sensor(Sensor::RANGE, i);
        m_rangeOut[i] = new OutPort<TimedDoubleSeq>(s->name.c_str(), m_range[i]);
        addOutPort(s->name.c_str(), *m_rangeOut[i]);
    }

    int ncamera = numSensors(Sensor::VISION);
    m_image.resize(ncamera);
    m_imageOut.resize(ncamera);
    for (int id=0; id<ncamera; id++){
        VisionSensor *s = sensor<VisionSensor>(id);
        if (!s) continue;
        if (s->imageType == VisionSensor::COLOR 
            || s->imageType == VisionSensor::COLOR_DEPTH){
            m_image[id].data.image.width = s->width;
            m_image[id].data.image.height = s->height;
            m_image[id].data.image.format = Img::CF_RGB;
            int len = s->width*s->height*3;
            m_image[id].data.image.raw_data.length(len);
            m_imageOut[id] = new OutPort<Img::TimedCameraImage>(s->name.c_str(), m_image[id]);
            addOutPort(s->name.c_str(), *m_imageOut[id]);
        }else if(s->imageType == VisionSensor::MONO
                 || s->imageType == VisionSensor::MONO_DEPTH){
            m_image[id].data.image.width = s->width;
            m_image[id].data.image.height = s->height;
            m_image[id].data.image.format = Img::CF_GRAY;
            int len = s->width*s->height;
            m_image[id].data.image.raw_data.length(len);
            m_imageOut[id] = new OutPort<Img::TimedCameraImage>(s->name.c_str(), m_image[id]);
            addOutPort(s->name.c_str(), *m_imageOut[id]);
        }
        if (s->imageType == VisionSensor::DEPTH
            || s->imageType == VisionSensor::COLOR_DEPTH
            || s->imageType == VisionSensor::MONO_DEPTH){
        }
    }
}

void BodyRTC::writeDataPorts()
{
    if (numJoints()){
        for (int i=0; i<numJoints(); i++){
            Link *l = joint(i);
            if (l){
                m_q.data[l->jointId] = l->q;
                m_dq.data[l->jointId] = l->dq;
            }
        }
        m_qOut.write();
        m_dqOut.write();
    }

    Link *root = rootLink();

    m_basePose.data.position.x = root->p[0];
    m_basePose.data.position.y = root->p[1];
    m_basePose.data.position.z = root->p[2];
    hrp::Vector3 rpy;
    rpy = rpyFromRot(root->R);
    m_basePose.data.orientation.r = rpy[0];
    m_basePose.data.orientation.p = rpy[1];
    m_basePose.data.orientation.y = rpy[2];
    m_basePoseOut.write();

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

    n = numSensors(Sensor::RANGE);
    for (int id=0; id < n; ++id){
        RangeSensor *s = sensor<RangeSensor>(id);
        if (s->isUpdated){
            m_range[id].data.length(s->distances.size());
            memcpy(m_range[id].data.get_buffer(), &(s->distances[0]), 
                   sizeof(double)*s->distances.size());
            m_rangeOut[id]->write();
            s->isUpdated = false;
        }
    }

    n = numSensors(Sensor::VISION);
    for (int id=0; id < n; ++id){
        VisionSensor *s = sensor<VisionSensor>(id);
        if (s->isUpdated){
            if (s->imageType == VisionSensor::COLOR 
                || s->imageType == VisionSensor::MONO
                || s->imageType == VisionSensor::COLOR_DEPTH 
                || s->imageType == VisionSensor::MONO_DEPTH){
                if (m_image[id].data.image.raw_data.length() != s->image.size()){
                    std::cerr << "BodyRTC: mismatch image length " 
                              << m_image[id].data.image.raw_data.length()
                              << "<->" << s->image.size() << std::endl;
                }else{
                    memcpy(m_image[id].data.image.raw_data.get_buffer(), 
                           &s->image[0], s->image.size());
                    m_imageOut[id]->write();
#if 0
                    char filename[20];
                    sprintf(filename, "camera%d.ppm", s->id);
                    std::ofstream ofs(filename, std::ios::out | std::ios::trunc | std::ios::binary );
                    char buf[10];
                    unsigned char *pixels = &s->image[0];
                    sprintf(buf, "%d %d", s->width, s->height);
                    if (s->imageType == VisionSensor::COLOR
                        || s->imageType == VisionSensor::COLOR_DEPTH){
                        ofs << "P6";
                    }else{ 
                        ofs << "P5";
                    }
                    ofs << std::endl << buf << std::endl << "255" << std::endl;
                    ofs.write((char *)pixels, s->image.size());
#endif    
                }
            }else if (s->imageType == VisionSensor::DEPTH
                      || s->imageType == VisionSensor::COLOR_DEPTH 
                      || s->imageType == VisionSensor::MONO_DEPTH){
                // TODO : generate point cloud
            }
            s->isUpdated = false;
        }
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
