#include <iostream>
#include <hrpModel/Sensor.h>
#include <hrpModel/Link.h>
#include "BodyRTC.h"
#include "PortHandler.h"

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
      dummy(0)
{
    //std::cout << "constructor of BodyRTC"  << std::endl;
}

BodyRTC::~BodyRTC(void)
{
    //std::cout << "destructor of BodyRTC"  << std::endl;
    for (size_t i=0; i<m_inports.size(); i++){
        delete m_inports[i];
    }
    for (size_t i=0; i<m_outports.size(); i++){
        delete m_outports[i];
    }
}

void BodyRTC::writeDataPorts(double time)
{
    for (size_t i=0; i<m_outports.size(); i++){
        m_outports[i]->update(time);
    }
}

void BodyRTC::readDataPorts()
{
    for (size_t i=0; i<m_inports.size(); i++){
        m_inports[i]->update();
    }
}

void parsePortConfig(const std::string &config, 
                     std::string &name, std::string &type,
                     std::vector<std::string> &elements)
{
    std::string::size_type colon = 0, start=0; 
    colon = config.find(':', start);
    if (colon == std::string::npos){
        std::cerr << "can't find the first separator in [" << config << "]" 
                  << std::endl;
        return;
    }
    name = config.substr(start, colon);
    start = colon+1;
    colon = config.find(':', start);
    if (colon == std::string::npos){
        type = config.substr(start);
        return;
    }
    std::string elist = config.substr(start, colon-start);
    std::string::size_type comma;
    start = 0;
    comma = elist.find(',', start);
    while (comma != std::string::npos){
        std::string e = elist.substr(start, comma-start);
        elements.push_back(e);
        start = comma+1;
        comma = elist.find(',', start);
    }
    elements.push_back(elist.substr(start));
    start = colon+1;
    type = config.substr(start);
}

bool getJointList(hrp::Body *body, const std::vector<std::string> &elements,
                  std::vector<hrp::Link *> &joints)
{
    if (elements.size() == 0){
        for (int i=0; i<body->numJoints(); i++){
            joints.push_back(body->joint(i));
        }
    }else{
        for (size_t i=0; i<elements.size(); i++){
            hrp::Link *j = body->link(elements[i]);
            if (j){
                joints.push_back(j);
            }else{
                std::cerr << "can't find a joint(" << elements[i] << ")"
                          << std::endl;
                return false;
            }
        }
    }
    return true;
}

void BodyRTC::createInPort(const std::string &config)
{
    std::string name, type;
    std::vector<std::string> elements;
    parsePortConfig(config, name, type, elements);
    if (type == "JOINT_VALUE"){
        std::vector<hrp::Link *> joints;
        if (getJointList(this, elements, joints)){
            m_inports.push_back(
                new JointValueInPortHandler(this, name.c_str(), joints));
        }
    }else if(type == "JOINT_VELOCITY"){
        std::vector<hrp::Link *> joints;
        if (getJointList(this, elements, joints)){
            m_inports.push_back(
                new JointVelocityInPortHandler(this, name.c_str(), joints));
        }
    }else if(type == "JOINT_ACCELERATION"){
        std::vector<hrp::Link *> joints;
        if (getJointList(this, elements, joints)){
            m_inports.push_back(
                new JointAccelerationInPortHandler(this, name.c_str(),joints));
        }
    }else if(type == "JOINT_TORQUE"){
        std::vector<hrp::Link *> joints;
        if (getJointList(this, elements, joints)){
            m_inports.push_back(
                new JointTorqueInPortHandler(this, name.c_str(), joints));
        }
    }else if(type == "EXTERNAL_FORCE"){
        std::cout << "EXTERNAL_FORCE is not implemented yet" << std::endl;
    }else if(type == "ABS_TRANSFORM"){
        std::vector<hrp::Link *> joints;
        if (getJointList(this, elements, joints) && joints.size() == 1){
            m_inports.push_back(
                new AbsTransformInPortHandler(this, name.c_str(), joints[0]));
        }
    }else if(type == "ABS_VELOCITY"){
        std::vector<hrp::Link *> joints;
        if (getJointList(this, elements, joints) && joints.size() == 1){
            m_inports.push_back(
                new AbsVelocityInPortHandler(this, name.c_str(), joints[0]));
        }
    }else if(type == "ABS_ACCELERATION"){
        std::vector<hrp::Link *> joints;
        if (getJointList(this, elements, joints) && joints.size() == 1){
            m_inports.push_back(
                new AbsAccelerationInPortHandler(this,name.c_str(),joints[0]));
        }
    }else if(type == "FRAME_RATE"){
        VisionSensor *s = this->sensor<VisionSensor>(elements[0]);
        if (!s){
            std::cerr << "can't find a sensor(" << elements[0] << ")" 
                      << std::endl;
            return;
        }
        m_inports.push_back(new FrameRateInPortHandler(this,name.c_str(),s));
    }else if(type == "SWITE_SWITCH"){
        Light *l = this->light(elements[0]);
        if (!l){
            std::cerr << "can't find a light(" << elements[0] << ")" 
                      << std::endl;
            return;
        }
        m_inports.push_back(new LightSwitchInPortHandler(this,name.c_str(),l));
    }else{
        std::cerr << "unknown InPort data type(" << type << ")" << std::endl;
    }
}

void BodyRTC::createOutPort(const std::string &config)
{
    std::string name, type;
    std::vector<std::string> elements;
    parsePortConfig(config, name, type, elements);
    if (type == "JOINT_VALUE"){
        std::vector<hrp::Link *> joints;
        if (getJointList(this, elements, joints)){
            m_outports.push_back(
                new JointValueOutPortHandler(this, name.c_str(), joints));
        }
    }else if(type == "JOINT_VELOCITY"){
        std::vector<hrp::Link *> joints;
        if (getJointList(this, elements, joints)){
            m_outports.push_back(
                new JointVelocityOutPortHandler(this, name.c_str(), joints));
        }
    }else if(type == "JOINT_ACCELERATION"){
        std::vector<hrp::Link *> joints;
        if (getJointList(this, elements, joints)){
            m_outports.push_back(
                new JointAccelerationOutPortHandler(this,name.c_str(),joints));
        }
    }else if(type == "JOINT_TORQUE"){
        std::vector<hrp::Link *> joints;
        if (getJointList(this, elements, joints)){
            m_outports.push_back(
                new JointTorqueOutPortHandler(this, name.c_str(), joints));
        }
    }else if(type == "ABS_TRANSFORM"){
        if (elements.size()!=1){
            std::cerr << "link name is not specified for port " << name 
                      << std::endl;
            return;
        }
        hrp::Link *l=this->link(elements[0]);
        if (l){
            m_outports.push_back(
                new AbsTransformOutPortHandler(this, name.c_str(), l));
            return;
        }
        hrp::Sensor *s;
        s = this->sensor<AccelSensor>(elements[0]);
        if (!s) s = this->sensor<RateGyroSensor>(elements[0]);
        if (!s) s = this->sensor<ForceSensor>(elements[0]);
        if (!s) s = this->sensor<RangeSensor>(elements[0]);
        if (!s) s = this->sensor<VisionSensor>(elements[0]);
        if (s){
            m_outports.push_back(
                new AbsTransformOutPortHandler(this, name.c_str(), s));
            return;
        }
        std::cerr << "can't find a link(or a sensor)(" << elements[0] << ")" 
                  << std::endl;
    }else if(type == "ABS_VELOCITY"){
        if (elements.size()!=1){
            std::cerr << "link name is not specified for port " << name
                      << std::endl;
            return;
        }
        hrp::Link *l=this->link(elements[0]);
        if (l){
            m_outports.push_back(
                new AbsVelocityOutPortHandler(this, name.c_str(), l));
        }else{
            std::cerr << "can't find a link(" << elements[0] << ")" 
                      << std::endl;
        }
    }else if(type == "ABS_ACCELERATION"){
        if (elements.size()!=1){
            std::cerr << "link name is not specified for port " << name
                      << std::endl;
            return;
        }
        hrp::Link *l=this->link(elements[0]);
        if (l){
            m_outports.push_back(
                new AbsAccelerationOutPortHandler(this, name.c_str(), l));
        }else{
            std::cerr << "can't find a link(" << elements[0] << ")" 
                      << std::endl;
        }
    }else if(type == "FORCE_SENSOR"){
        if (elements.size()!=1){
            std::cerr << "sensor name is not specified for port" << name 
                      << std::endl;
            return;
        }
        ForceSensor *s = this->sensor<ForceSensor>(elements[0]);
        if (!s){
            std::cerr << "can't find a sensor(" << elements[0] << ")" 
                      << std::endl;
            return;
        }
        m_outports.push_back(new ForceSensorPortHandler(this, name.c_str(),s));
                                                        
    }else if(type == "RATE_GYRO_SENSOR"){
        if (elements.size()!=1){
            std::cerr << "sensor name is not specified for port " << name
                      << std::endl;
            return;
        }
        RateGyroSensor *s = this->sensor<RateGyroSensor>(elements[0]);
        if (!s){
            std::cerr << "can't find a sensor(" << elements[0] << ")" 
                      << std::endl;
            return;
        }
        m_outports.push_back(new RateGyroSensorPortHandler(this, name.c_str(),
                                                           s));
    }else if(type == "ACCELERATION_SENSOR"){
        if (elements.size()!=1){
            std::cerr << "sensor name is not specified for port " << name
                      << std::endl;
            return;
        }
        AccelSensor *s = this->sensor<AccelSensor>(elements[0]);
        if (!s){
            std::cerr << "can't find a sensor(" << elements[0] << ")" 
                      << std::endl;
            return;
        }
        m_outports.push_back(new AccelSensorPortHandler(this, name.c_str(),s));
                                                        
    }else if(type == "RANGE_SENSOR"){
        if (elements.size()!=1){
            std::cerr << "sensor name is not specified for port " << name 
                      << std::endl;
            return;
        }
        RangeSensor *s = this->sensor<RangeSensor>(elements[0]);
        if (!s){
            std::cerr << "can't find a sensor(" << elements[0] << ")" 
                      << std::endl;
            return;
        }
        m_outports.push_back(new RangeSensorPortHandler(this, name.c_str(),s));
                                                        
    }else if(type == "VISION_SENSOR"){
        if (elements.size()!=1){
            std::cerr << "sensor name is not specified for port " << name
                      << std::endl;
            return;
        }
        VisionSensor *s = this->sensor<VisionSensor>(elements[0]);
        if (!s){
            std::cerr << "can't find a sensor(" << elements[0] << ")" 
                      << std::endl;
            return;
        }
        m_outports.push_back(new VisionSensorPortHandler(this,name.c_str(),s));
    }else if(type == "POINT_CLOUD"){
        if (elements.size()!=1){
            std::cerr << "sensor name is not specified for port " << name
                      << std::endl;
            return;
        }
        VisionSensor *s = this->sensor<VisionSensor>(elements[0]);
        if (!s){
            std::cerr << "can't find a sensor(" << elements[0] << ")" 
                      << std::endl;
            return;
        }
        m_outports.push_back(new PointCloudPortHandler(this,name.c_str(),s));
    }else if(type == "CONSTRAINT_FORCE"){
        std::cout << "CONSTRAINT_FORCE is not implemented yet" << std::endl;
    }else{
        std::cerr << "unknown InPort data type(" << type << ")" << std::endl;
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
