#include <hrpModel/Link.h>
#include <hrpModel/Sensor.h>
#include "PortHandler.h"

using namespace hrp;

JointInPortHandler::JointInPortHandler(
    RTC::DataFlowComponentBase *i_rtc, 
    const char *i_portName,
    const std::vector<Link *> &i_joints) : 
    InPortHandler<RTC::TimedDoubleSeq>(i_rtc, i_portName),
    m_joints(i_joints)
{
    m_data.data.length(m_joints.size());
}

JointOutPortHandler::JointOutPortHandler(
    RTC::DataFlowComponentBase *i_rtc, 
    const char *i_portName,
    const std::vector<Link *> &i_joints) : 
    OutPortHandler<RTC::TimedDoubleSeq>(i_rtc, i_portName),
    m_joints(i_joints)
{
    m_data.data.length(m_joints.size());
}

JointValueInPortHandler::JointValueInPortHandler(
    RTC::DataFlowComponentBase *i_rtc, 
    const char *i_portName,
    const std::vector<Link *> &i_joints) : 
    JointInPortHandler(i_rtc, i_portName, i_joints)
{
}

void JointValueInPortHandler::update()
{
    if (m_port.isNew()){
        do {
            m_port.read();
        }while(m_port.isNew());
        for (size_t i=0; i<m_joints.size(); i++){
            if (m_joints[i]) m_joints[i]->q = m_data.data[i];
        }
    }
}

JointValueOutPortHandler::JointValueOutPortHandler(
    RTC::DataFlowComponentBase *i_rtc, 
    const char *i_portName,
    const std::vector<Link *> &i_joints) : 
    JointOutPortHandler(i_rtc, i_portName, i_joints)
{
}

void JointValueOutPortHandler::update()
{
    for (size_t i=0; i<m_joints.size(); i++){
        if (m_joints[i]) m_data.data[i] = m_joints[i]->q;
    }
    m_port.write();
}

JointVelocityInPortHandler::JointVelocityInPortHandler(
    RTC::DataFlowComponentBase *i_rtc, 
    const char *i_portName,
    const std::vector<Link *> &i_joints) : 
    JointInPortHandler(i_rtc, i_portName, i_joints)
{
}

void JointVelocityInPortHandler::update()
{
    if (m_port.isNew()){
        do{
            m_port.read();
        }while(m_port.isNew());
        for (size_t i=0; i<m_joints.size(); i++){
            if (m_joints[i]) m_joints[i]->dq = m_data.data[i];
        }
    }
}

JointVelocityOutPortHandler::JointVelocityOutPortHandler(
    RTC::DataFlowComponentBase *i_rtc, 
    const char *i_portName,
    const std::vector<Link *> &i_joints) : 
    JointOutPortHandler(i_rtc, i_portName, i_joints)
{
}

void JointVelocityOutPortHandler::update()
{
    for (size_t i=0; i<m_joints.size(); i++){
        if (m_joints[i]) m_data.data[i] = m_joints[i]->dq;
    }
    m_port.write();
}

JointAccelerationInPortHandler::JointAccelerationInPortHandler(
    RTC::DataFlowComponentBase *i_rtc, 
    const char *i_portName,
    const std::vector<Link *> &i_joints) : 
    JointInPortHandler(i_rtc, i_portName, i_joints)
{
}

void JointAccelerationInPortHandler::update()
{
    if (m_port.isNew()){
        do{
            m_port.read();
        }while(m_port.isNew());
        for (size_t i=0; i<m_joints.size(); i++){
            if (m_joints[i]) m_joints[i]->ddq = m_data.data[i];
        }
    }
}

JointAccelerationOutPortHandler::JointAccelerationOutPortHandler(
    RTC::DataFlowComponentBase *i_rtc, 
    const char *i_portName,
    const std::vector<Link *> &i_joints) : 
    JointOutPortHandler(i_rtc, i_portName, i_joints)
{
}

void JointAccelerationOutPortHandler::update()
{
    for (size_t i=0; i<m_joints.size(); i++){
        if (m_joints[i]) m_data.data[i] = m_joints[i]->ddq;
    }
    m_port.write();
}

JointTorqueInPortHandler::JointTorqueInPortHandler(
    RTC::DataFlowComponentBase *i_rtc, 
    const char *i_portName,
    const std::vector<Link *> &i_joints) : 
    JointInPortHandler(i_rtc, i_portName, i_joints)
{
}

void JointTorqueInPortHandler::update()
{
    if (m_port.isNew()){
        do{
            m_port.read();
        }while(m_port.isNew());
        for (size_t i=0; i<m_joints.size(); i++){
            if (m_joints[i]) m_joints[i]->u = m_data.data[i];
        }
    }
}

JointTorqueOutPortHandler::JointTorqueOutPortHandler(
    RTC::DataFlowComponentBase *i_rtc, 
    const char *i_portName,
    const std::vector<Link *> &i_joints) : 
    JointOutPortHandler(i_rtc, i_portName, i_joints)
{
}

void JointTorqueOutPortHandler::update()
{
    for (size_t i=0; i<m_joints.size(); i++){
        if (m_joints[i]) m_data.data[i] = m_joints[i]->u;
    }
    m_port.write();
}

ForceSensorPortHandler::ForceSensorPortHandler(
    RTC::DataFlowComponentBase *i_rtc, 
    const char *i_portName,
    ForceSensor *i_sensor) : 
    SensorPortHandler<ForceSensor, RTC::TimedDoubleSeq>(i_rtc, i_portName, i_sensor)
{
    m_data.data.length(6);
}

void ForceSensorPortHandler::update()
{
    setVector3(m_sensor->f,   m_data.data, 0);
    setVector3(m_sensor->tau, m_data.data, 3);
    m_port.write();
}

RateGyroSensorPortHandler::RateGyroSensorPortHandler(
    RTC::DataFlowComponentBase *i_rtc, 
    const char *i_portName,
    RateGyroSensor *i_sensor) : 
    SensorPortHandler<RateGyroSensor, RTC::TimedAngularVelocity3D>(i_rtc, i_portName, i_sensor)
{
}

void RateGyroSensorPortHandler::update()
{
    m_data.data.avx = m_sensor->w[0];
    m_data.data.avy = m_sensor->w[1];
    m_data.data.avz = m_sensor->w[2];
    m_port.write();
}

AccelSensorPortHandler::AccelSensorPortHandler(
    RTC::DataFlowComponentBase *i_rtc, 
    const char *i_portName,
    AccelSensor *i_sensor) : 
    SensorPortHandler<AccelSensor, RTC::TimedAcceleration3D>(i_rtc, i_portName, i_sensor)
{
}

void AccelSensorPortHandler::update()
{
    m_data.data.ax = m_sensor->dv[0];
    m_data.data.ay = m_sensor->dv[1];
    m_data.data.az = m_sensor->dv[2];
    m_port.write();
}

RangeSensorPortHandler::RangeSensorPortHandler(
    RTC::DataFlowComponentBase *i_rtc, 
    const char *i_portName,
    RangeSensor *i_sensor) : 
    SensorPortHandler<RangeSensor, RTC::TimedDoubleSeq>(i_rtc, i_portName, i_sensor)
{
    i_sensor->isEnabled = true;
}

void RangeSensorPortHandler::update()
{
    if (m_sensor->isUpdated){
        if (m_data.data.length() != m_sensor->distances.size()){
            m_data.data.length(m_sensor->distances.size());
        }
        memcpy(m_data.data.get_buffer(), &(m_sensor->distances[0]), 
               sizeof(double)*m_sensor->distances.size());
        m_port.write();
        m_sensor->isUpdated = false;
    }
}


VisionSensorPortHandler::VisionSensorPortHandler(
    RTC::DataFlowComponentBase *i_rtc, 
    const char *i_portName,
    VisionSensor *i_sensor) : 
    SensorPortHandler<VisionSensor, Img::TimedCameraImage>(i_rtc, i_portName, i_sensor)
{
    i_sensor->isEnabled = true;
    if (m_sensor->imageType == VisionSensor::COLOR 
        || m_sensor->imageType == VisionSensor::COLOR_DEPTH){
        m_data.data.image.width = m_sensor->width;
        m_data.data.image.height = m_sensor->height;
        m_data.data.image.format = Img::CF_RGB;
        int len = m_sensor->width*m_sensor->height*3;
        m_data.data.image.raw_data.length(len);
    }else if(m_sensor->imageType == VisionSensor::MONO
             || m_sensor->imageType == VisionSensor::MONO_DEPTH){
        m_data.data.image.width = m_sensor->width;
        m_data.data.image.height = m_sensor->height;
        m_data.data.image.format = Img::CF_GRAY;
        int len = m_sensor->width*m_sensor->height;
        m_data.data.image.raw_data.length(len);
    }
    if (m_sensor->imageType == VisionSensor::DEPTH
        || m_sensor->imageType == VisionSensor::COLOR_DEPTH
        || m_sensor->imageType == VisionSensor::MONO_DEPTH){
    }
}

void VisionSensorPortHandler::update()
{
    if (m_sensor->isUpdated){
        if (m_sensor->imageType == VisionSensor::COLOR 
            || m_sensor->imageType == VisionSensor::MONO
            || m_sensor->imageType == VisionSensor::COLOR_DEPTH 
            || m_sensor->imageType == VisionSensor::MONO_DEPTH){
            if (m_data.data.image.raw_data.length() != m_sensor->image.size()){
                std::cerr << "BodyRTC: mismatch image length " 
                          << m_data.data.image.raw_data.length()
                          << "<->" << m_sensor->image.size() << std::endl;
            }else{
                memcpy(m_data.data.image.raw_data.get_buffer(), 
                       &m_sensor->image[0], m_sensor->image.size());
                m_port.write();
#if 0
                char filename[20];
                sprintf(filename, "camera%d.ppm", m_sensor->id);
                std::ofstream ofs(filename, std::ios::out | std::ios::trunc | std::ios::binary );
                char buf[10];
                unsigned char *pixels = &m_sensor->image[0];
                sprintf(buf, "%d %d", m_sensor->width, m_sensor->height);
                if (m_sensor->imageType == VisionSensor::COLOR
                    || m_sensor->imageType == VisionSensor::COLOR_DEPTH){
                    ofs << "P6";
                }else{ 
                    ofs << "P5";
                }
                ofs << std::endl << buf << std::endl << "255" << std::endl;
                ofs.write((char *)pixels, m_sensor->image.size());
#endif    
            }
        }else if (m_sensor->imageType == VisionSensor::DEPTH
                  || m_sensor->imageType == VisionSensor::COLOR_DEPTH 
                  || m_sensor->imageType == VisionSensor::MONO_DEPTH){
            // TODO : generate point cloud
        }
        m_sensor->isUpdated = false;
    }
}


