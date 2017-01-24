#include <hrpModel/Link.h>
#include <hrpModel/Sensor.h>
#include <hrpModel/Light.h>
#include "PortHandler.h"

using namespace hrp;

JointInPortHandler::JointInPortHandler(
    RTC::DataFlowComponentBase *i_rtc, 
    const char *i_portName,
    const std::vector<Link *> &i_joints,
    std::vector<OpenHRP::RobotHardwareService::SwitchStatus> *i_servo) :
    InPortHandler<RTC::TimedDoubleSeq>(i_rtc, i_portName),
    m_joints(i_joints),
    m_servo(*i_servo)
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
    const std::vector<Link *> &i_joints,
    std::vector<OpenHRP::RobotHardwareService::SwitchStatus> *i_servo) :
    JointInPortHandler(i_rtc, i_portName, i_joints, i_servo)
{
}

void JointValueInPortHandler::update()
{
    if (m_port.isNew()){
        do {
            m_port.read();
        }while(m_port.isNew());
        for (size_t i=0; i<m_joints.size(); i++){
            if (m_joints[i] && m_servo[i] == OpenHRP::RobotHardwareService::SWITCH_ON) m_joints[i]->q = m_data.data[i];
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

void JointValueOutPortHandler::update(double time)
{
    for (size_t i=0; i<m_joints.size(); i++){
        if (m_joints[i]) m_data.data[i] = m_joints[i]->q;
    }
    write(time);
}

JointVelocityInPortHandler::JointVelocityInPortHandler(
    RTC::DataFlowComponentBase *i_rtc, 
    const char *i_portName,
    const std::vector<Link *> &i_joints,
    std::vector<OpenHRP::RobotHardwareService::SwitchStatus> *i_servo) :
    JointInPortHandler(i_rtc, i_portName, i_joints, i_servo)
{
}

void JointVelocityInPortHandler::update()
{
    if (m_port.isNew()){
        do{
            m_port.read();
        }while(m_port.isNew());
        for (size_t i=0; i<m_joints.size(); i++){
            if (m_joints[i] && m_servo[i] == OpenHRP::RobotHardwareService::SWITCH_ON) m_joints[i]->dq = m_data.data[i];
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

void JointVelocityOutPortHandler::update(double time)
{
    for (size_t i=0; i<m_joints.size(); i++){
        if (m_joints[i]) m_data.data[i] = m_joints[i]->dq;
    }
    write(time);
}

JointAccelerationInPortHandler::JointAccelerationInPortHandler(
    RTC::DataFlowComponentBase *i_rtc, 
    const char *i_portName,
    const std::vector<Link *> &i_joints,
    std::vector<OpenHRP::RobotHardwareService::SwitchStatus> *i_servo) :
    JointInPortHandler(i_rtc, i_portName, i_joints, i_servo)
{
}

void JointAccelerationInPortHandler::update()
{
    if (m_port.isNew()){
        do{
            m_port.read();
        }while(m_port.isNew());
        for (size_t i=0; i<m_joints.size(); i++){
            if (m_joints[i] && m_servo[i] == OpenHRP::RobotHardwareService::SWITCH_ON) m_joints[i]->ddq = m_data.data[i];
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

void JointAccelerationOutPortHandler::update(double time)
{
    for (size_t i=0; i<m_joints.size(); i++){
        if (m_joints[i]) m_data.data[i] = m_joints[i]->ddq;
    }
    write(time);
}

JointTorqueInPortHandler::JointTorqueInPortHandler(
    RTC::DataFlowComponentBase *i_rtc, 
    const char *i_portName,
    const std::vector<Link *> &i_joints,
    std::vector<OpenHRP::RobotHardwareService::SwitchStatus> *i_servo) :
    JointInPortHandler(i_rtc, i_portName, i_joints, i_servo)
{
}

void JointTorqueInPortHandler::update()
{
    if (m_port.isNew()){
        do{
            m_port.read();
        }while(m_port.isNew());
        if (m_data.data.length() != m_joints.size()){
            std::cerr << "JointTorqueInPortHandler: data length mismatch(length of input data:"
                      << m_data.data.length() << "<->the number of joints:" << m_joints.size()
                      << ")" << std::endl;
        } 
        for (size_t i=0; i<m_joints.size(); i++){
            if (m_joints[i] && m_servo[i] == OpenHRP::RobotHardwareService::SWITCH_ON) m_joints[i]->u = m_data.data[i];
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

void JointTorqueOutPortHandler::update(double time)
{
    for (size_t i=0; i<m_joints.size(); i++){
        if (m_joints[i]) m_data.data[i] = m_joints[i]->u;
    }
    write(time);
}

ForceSensorPortHandler::ForceSensorPortHandler(
    RTC::DataFlowComponentBase *i_rtc, 
    const char *i_portName,
    ForceSensor *i_sensor) : 
    SensorPortHandler<ForceSensor, RTC::TimedDoubleSeq>(i_rtc, i_portName, i_sensor)
{
    m_data.data.length(6);
}

void ForceSensorPortHandler::update(double time)
{
    setVector3(m_sensor->f,   m_data.data, 0);
    setVector3(m_sensor->tau, m_data.data, 3);
    write(time);
}

RateGyroSensorPortHandler::RateGyroSensorPortHandler(
    RTC::DataFlowComponentBase *i_rtc, 
    const char *i_portName,
    RateGyroSensor *i_sensor) : 
    SensorPortHandler<RateGyroSensor, RTC::TimedAngularVelocity3D>(i_rtc, i_portName, i_sensor)
{
}

void RateGyroSensorPortHandler::update(double time)
{
    m_data.data.avx = m_sensor->w[0];
    m_data.data.avy = m_sensor->w[1];
    m_data.data.avz = m_sensor->w[2];
    write(time);
}

AccelSensorPortHandler::AccelSensorPortHandler(
    RTC::DataFlowComponentBase *i_rtc, 
    const char *i_portName,
    AccelSensor *i_sensor) : 
    SensorPortHandler<AccelSensor, RTC::TimedAcceleration3D>(i_rtc, i_portName, i_sensor)
{
}

void AccelSensorPortHandler::update(double time)
{
    m_data.data.ax = m_sensor->dv[0];
    m_data.data.ay = m_sensor->dv[1];
    m_data.data.az = m_sensor->dv[2];
    write(time);
}

RangeSensorPortHandler::RangeSensorPortHandler(
    RTC::DataFlowComponentBase *i_rtc, 
    const char *i_portName,
    RangeSensor *i_sensor) : 
    SensorPortHandler<RangeSensor, RTC::RangeData>(i_rtc, i_portName, i_sensor)
{
    i_sensor->isEnabled = true;
    m_data.config.minAngle = -i_sensor->scanAngle/2;
    m_data.config.maxAngle =  i_sensor->scanAngle/2;
    m_data.config.angularRes = i_sensor->scanStep;
    m_data.config.minRange = 0;
    m_data.config.maxRange = i_sensor->maxDistance;
    m_data.config.rangeRes = 0;
    m_data.config.frequency = i_sensor->scanRate;
}

void RangeSensorPortHandler::update(double time)
{
    if (m_sensor->isUpdated){
        if (m_data.ranges.length() != m_sensor->distances.size()){
            m_data.ranges.length(m_sensor->distances.size());
        }
        memcpy(m_data.ranges.get_buffer(), &(m_sensor->distances[0]), 
               sizeof(double)*m_sensor->distances.size());
        write(time);
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
        m_data.data.intrinsic.distortion_coefficient.length(5);
        for(int i = 0; i < 5; i++){
            m_data.data.intrinsic.distortion_coefficient[i] = 0;
        }
        double fovx = m_sensor->width/m_sensor->height*m_sensor->fovy;
        m_data.data.intrinsic.matrix_element[0]=0.5 * m_sensor->width/tan(fovx/2.0);
        m_data.data.intrinsic.matrix_element[1]=0.0;
        m_data.data.intrinsic.matrix_element[2]=m_sensor->width/2;
        m_data.data.intrinsic.matrix_element[3]=0.5 * m_sensor->height/tan(m_sensor->fovy/2.0);
        m_data.data.intrinsic.matrix_element[4]=m_sensor->height/2;
    }else if(m_sensor->imageType == VisionSensor::MONO
             || m_sensor->imageType == VisionSensor::MONO_DEPTH){
        m_data.data.image.width = m_sensor->width;
        m_data.data.image.height = m_sensor->height;
        m_data.data.image.format = Img::CF_GRAY;
        int len = m_sensor->width*m_sensor->height;
        m_data.data.image.raw_data.length(len);
        m_data.data.intrinsic.distortion_coefficient.length(5);
        for(int i = 0; i < 5; i++){
            m_data.data.intrinsic.distortion_coefficient[i] = 0;
        }
        double fovx = m_sensor->width/m_sensor->height*m_sensor->fovy;
        m_data.data.intrinsic.matrix_element[0]=0.5 * m_sensor->width/tan(fovx/2.0);
        m_data.data.intrinsic.matrix_element[1]=0.0;
        m_data.data.intrinsic.matrix_element[2]=m_sensor->width/2;
        m_data.data.intrinsic.matrix_element[3]=0.5 * m_sensor->height/tan(m_sensor->fovy/2.0);
        m_data.data.intrinsic.matrix_element[4]=m_sensor->height/2;
    }
}

void VisionSensorPortHandler::update(double time)
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
                write(time);
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
        }
        m_sensor->isUpdated = false;
    }
}

PointCloudPortHandler::PointCloudPortHandler(
    RTC::DataFlowComponentBase *i_rtc, 
    const char *i_portName,
    VisionSensor *i_sensor) : 
    SensorPortHandler<VisionSensor, PointCloudTypes::PointCloud>(i_rtc, i_portName, i_sensor)
{
    i_sensor->isEnabled = true;
    switch(m_sensor->imageType){
    case VisionSensor::DEPTH:
        m_pcFormat = "xyz"; break;
    case VisionSensor::COLOR_DEPTH:
        m_pcFormat = "xyzrgb"; break;
    case VisionSensor::MONO_DEPTH:
        m_pcFormat = "xyz"; break;
    default:
        std::cout << "VisionSensor " << m_sensor->name
                  << " doesn't have distance measuring function" << std::endl;
        break;
    }
    m_data.width = m_sensor->width;
    m_data.height = m_sensor->height;
    m_data.type = m_pcFormat.c_str();

    bool colored = false;
    if (m_pcFormat == "xyz"){
        m_data.fields.length(3);
    }else if (m_pcFormat == "xyzrgb"){
        m_data.fields.length(6);
        colored = true;
    }else{
        std::cerr << "unknown point cloud format:[" << m_pcFormat << "]" << std::endl;
    }
    m_data.fields[0].name = "x";
    m_data.fields[0].offset = 0;
    m_data.fields[0].data_type = PointCloudTypes::FLOAT32;
    m_data.fields[0].count = 4;
    m_data.fields[1].name = "y";
    m_data.fields[1].offset = 4;
    m_data.fields[1].data_type = PointCloudTypes::FLOAT32;
    m_data.fields[1].count = 4;
    m_data.fields[2].name = "z";
    m_data.fields[2].offset = 8;
    m_data.fields[2].data_type = PointCloudTypes::FLOAT32;
    m_data.fields[2].count = 4;
    if (m_pcFormat == "xyzrgb"){
        m_data.fields[3].name = "r";
        m_data.fields[3].offset = 12;
        m_data.fields[3].data_type = PointCloudTypes::UINT8;
        m_data.fields[3].count = 1;
        m_data.fields[4].name = "g";
        m_data.fields[4].offset = 13;
        m_data.fields[4].data_type = PointCloudTypes::UINT8;
        m_data.fields[4].count = 1;
        m_data.fields[5].name = "b";
        m_data.fields[5].offset = 14;
        m_data.fields[5].data_type = PointCloudTypes::UINT8;
        m_data.fields[5].count = 1;
    }
    m_data.is_bigendian = false;
    m_data.point_step = 16;
    m_data.is_dense = true;
    m_data.row_step = m_data.point_step*m_sensor->width;
}

void PointCloudPortHandler::update(double time)
{
    if (m_sensor->isUpdated){
        m_data.data.length(m_sensor->depth.size());
        memcpy(m_data.data.get_buffer(), &m_sensor->depth[0],
               m_sensor->depth.size());
        write(time);
    }
    m_sensor->isUpdated = false;
}

AbsTransformInPortHandler::AbsTransformInPortHandler(
    RTC::DataFlowComponentBase *i_rtc,
    const char *i_portName,
    hrp::Link *i_link) :
    InPortHandler<RTC::TimedPose3D>(i_rtc, i_portName),
    m_link(i_link)
{
}

void AbsTransformInPortHandler::update()
{
    if (m_port.isNew()){
        do{
            m_port.read();
        }while(m_port.isNew());
        m_link->p << 
            m_data.data.position.x, 
            m_data.data.position.y, 
            m_data.data.position.z;
        hrp::Matrix33 R = hrp::rotFromRpy(m_data.data.orientation.r,
                                          m_data.data.orientation.p,
                                          m_data.data.orientation.y);
        m_link->setSegmentAttitude(R);
    }
}

AbsVelocityInPortHandler::AbsVelocityInPortHandler(
    RTC::DataFlowComponentBase *i_rtc,
    const char *i_portName,
    hrp::Link *i_link) :
    InPortHandler<RTC::TimedDoubleSeq>(i_rtc, i_portName),
    m_link(i_link)
{
}

void AbsVelocityInPortHandler::update()
{
    if (m_port.isNew()){
        do{
            m_port.read();
        }while(m_port.isNew());
        m_link->v << m_data.data[0], m_data.data[1], m_data.data[2];
        m_link->w << m_data.data[3], m_data.data[4], m_data.data[5];
        m_link->vo = m_link->v - m_link->w.cross(m_link->p);
    }
}

AbsAccelerationInPortHandler::AbsAccelerationInPortHandler(
    RTC::DataFlowComponentBase *i_rtc,
    const char *i_portName,
    hrp::Link *i_link) :
    InPortHandler<RTC::TimedDoubleSeq>(i_rtc, i_portName),
    m_link(i_link)
{
}

void AbsAccelerationInPortHandler::update()
{
    if (m_port.isNew()){
        do{
            m_port.read();
        }while(m_port.isNew());
        m_link->dv << m_data.data[0], m_data.data[1], m_data.data[2];
        m_link->dw << m_data.data[3], m_data.data[4], m_data.data[5];
    }
}

FrameRateInPortHandler::FrameRateInPortHandler(
    RTC::DataFlowComponentBase *i_rtc,
    const char *i_portName,
    hrp::VisionSensor *i_sensor) :
    InPortHandler<RTC::TimedDouble>(i_rtc, i_portName),
    m_sensor(i_sensor)
{
}

void FrameRateInPortHandler::update()
{
    if (m_port.isNew()){
        do {
            m_port.read();
        }while(m_port.isNew());
        m_sensor->frameRate = m_data.data;
    }
}

LightSwitchInPortHandler::LightSwitchInPortHandler(
    RTC::DataFlowComponentBase *i_rtc,
    const char *i_portName,
    hrp::Light *i_light) :
    InPortHandler<RTC::TimedBoolean>(i_rtc, i_portName),
    m_light(i_light)
{
}

void LightSwitchInPortHandler::update()
{
    if (m_port.isNew()){
        do {
            m_port.read();
        }while(m_port.isNew());
        m_light->on = m_data.data;
    }
}

AbsTransformOutPortHandler::AbsTransformOutPortHandler(
    RTC::DataFlowComponentBase *i_rtc,
    const char *i_portName,
    hrp::Link *i_link) :
    OutPortHandler<RTC::TimedPose3D>(i_rtc, i_portName),
    m_link(i_link), m_sensor(NULL)
{
}

AbsTransformOutPortHandler::AbsTransformOutPortHandler(
    RTC::DataFlowComponentBase *i_rtc,
    const char *i_portName,
    hrp::Sensor *i_sensor) :
    OutPortHandler<RTC::TimedPose3D>(i_rtc, i_portName),
    m_link(NULL), m_sensor(i_sensor)
{
}

void AbsTransformOutPortHandler::update(double time)
{
    hrp::Vector3 p;
    hrp::Matrix33 R;
    if (m_link){
        p = m_link->p;
        R = m_link->attitude();
    }else{
        hrp::Link *parent = m_sensor->link;
        p = parent->R*m_sensor->localPos+parent->p;
        R = parent->R*m_sensor->localR;
    }
    m_data.data.position.x = p[0];
    m_data.data.position.y = p[1];
    m_data.data.position.z = p[2];
    hrp::Vector3 rpy = rpyFromRot(R);
    m_data.data.orientation.r = rpy[0];
    m_data.data.orientation.p = rpy[1];
    m_data.data.orientation.y = rpy[2];
    write(time);
}


AbsVelocityOutPortHandler::AbsVelocityOutPortHandler(
    RTC::DataFlowComponentBase *i_rtc,
    const char *i_portName,
    hrp::Link *i_link) :
    OutPortHandler<RTC::TimedDoubleSeq>(i_rtc, i_portName),
    m_link(i_link)
{
    m_data.data.length(6);
}

void AbsVelocityOutPortHandler::update(double time){
    m_data.data[0] = m_link->v(0);
    m_data.data[1] = m_link->v(1);
    m_data.data[2] = m_link->v(2);
    m_data.data[3] = m_link->w(0);
    m_data.data[4] = m_link->w(1);
    m_data.data[5] = m_link->w(2);
}

AbsAccelerationOutPortHandler::AbsAccelerationOutPortHandler(
    RTC::DataFlowComponentBase *i_rtc,
    const char *i_portName,
    hrp::Link *i_link) :
    OutPortHandler<RTC::TimedDoubleSeq>(i_rtc, i_portName),
    m_link(i_link)
{
    m_data.data.length(6);
}

void AbsAccelerationOutPortHandler::update(double time)
{
    m_data.data[0] = m_link->dv(0);
    m_data.data[1] = m_link->dv(1);
    m_data.data[2] = m_link->dv(2);
    m_data.data[3] = m_link->dw(0);
    m_data.data[4] = m_link->dw(1);
    m_data.data[5] = m_link->dw(2);
}

EmergencySignalPortHandler::EmergencySignalPortHandler(
    RTC::DataFlowComponentBase *i_rtc,
    const char *i_portName,
    BodyRTC *i_body) :
    OutPortHandler<RTC::TimedLong>(i_rtc, i_portName),
    m_body(i_body)
{
}

void EmergencySignalPortHandler::update(double time)
{
    if (m_body->m_emergencyReason != BodyRTC::EMG_NONE){
        m_data.data = m_body->m_emergencyReason;
        write(time);
    }
}

ServoStatePortHandler::ServoStatePortHandler(
    RTC::DataFlowComponentBase *i_rtc,
    const char *i_portName,
    BodyRTC *i_body) :
    OutPortHandler<OpenHRP::TimedLongSeqSeq>(i_rtc, i_portName),
    m_body(i_body)
{
    rs = new OpenHRP::RobotHardwareService::RobotState();
}

void ServoStatePortHandler::update(double time)
{
    m_body->getStatus(rs);
    m_data.data.length(rs->servoState.length());
    for (size_t i=0; i < rs->servoState.length(); i++) {
        m_data.data[i].length(rs->servoState[i].length());
        for (size_t j=0; j < rs->servoState[i].length(); j++) {
            m_data.data[i][j] = rs->servoState[i][j];
        }
    }
    write(time);
}
