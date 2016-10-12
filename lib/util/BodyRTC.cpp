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
      m_RobotHardwareServicePort("RobotHardwareService"),
      m_resetPosition(true),
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

#define DEFAULT_ANGLE_ERROR_LIMIT 0.2 // [rad] // copied from robot.cpp
RTC::ReturnCode_t BodyRTC::setup(){
    std::cout << "BodyRTC::setup(), numJoints = " << numJoints() << std::endl;
    angles.resize(numJoints());
    commands.resize(numJoints());
    accels.resize(numSensors(hrp::Sensor::ACCELERATION));
    gyros.resize(numSensors(hrp::Sensor::RATE_GYRO));
    forces.resize(numSensors(hrp::Sensor::FORCE));
    calib_status.resize(numJoints());
    servo_status.resize(numJoints());
    power_status.resize(numJoints());
    m_servoErrorLimit.resize(numJoints());
    for(unsigned int i = 0; i < numJoints(); i++) {
        calib_status[i] = servo_status[i] = power_status[i] = OpenHRP::RobotHardwareService::SWITCH_ON;
        m_servoErrorLimit[i] = DEFAULT_ANGLE_ERROR_LIMIT;
    }
    m_emergencyReason = EMG_NONE; // clear
    m_emergencyId = -1;
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
        for (unsigned int i=0; i<body->numJoints(); i++){
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
                new JointValueInPortHandler(this, name.c_str(), joints, &servo_status));
        }
    }else if(type == "JOINT_VELOCITY"){
        std::vector<hrp::Link *> joints;
        if (getJointList(this, elements, joints)){
            m_inports.push_back(
                new JointVelocityInPortHandler(this, name.c_str(), joints, &servo_status));
        }
    }else if(type == "JOINT_ACCELERATION"){
        std::vector<hrp::Link *> joints;
        if (getJointList(this, elements, joints)){
            m_inports.push_back(
                new JointAccelerationInPortHandler(this, name.c_str(),joints, &servo_status));
        }
    }else if(type == "JOINT_TORQUE"){
        std::vector<hrp::Link *> joints;
        if (getJointList(this, elements, joints)){
            m_inports.push_back(
                new JointTorqueInPortHandler(this, name.c_str(), joints, &servo_status));
        }
    }else if(type == "EXTERNAL_FORCE"){
        std::cout << "EXTERNAL_FORCE is not implemented yet" << std::endl;
    }else if(type == "ABS_TRANSFORM"){
        std::vector<hrp::Link *> joints;
        if (getJointList(this, elements, joints) && joints.size() == 1){
            m_inports.push_back(
                new AbsTransformInPortHandler(this, name.c_str(), joints[0]));
        } else if (elements.size() == 1) {
            hrp::Link *l=this->link(elements[0]);
            if (l){
                m_inports.push_back(
                                     new AbsTransformInPortHandler(this, name.c_str(), l));
                return;
            }
            std::cerr << "can't find a link(or a sensor)(" << elements[0] << ")" 
                      << std::endl;
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
    }else if(type == "LIGHT_SWITCH"){
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
        std::cerr << "unknown OutPort data type(" << type << ")" << std::endl;
    }

    m_outports.push_back(new EmergencySignalPortHandler(this, "emergencySignal", this));
    m_outports.push_back(new ServoStatePortHandler(this, "servoState", this));

    m_service0.setRobot(this);
    m_RobotHardwareServicePort.registerProvider("service0", "RobotHardwareService", m_service0);
    addPort(m_RobotHardwareServicePort);
}

bool BodyRTC::names2ids(const std::vector<std::string> &i_names,
                        std::vector<int> &o_ids)
{
    bool ret = true;
    for (unsigned int i=0; i<i_names.size(); i++){
        hrp::Link *l = this->link(i_names[i].c_str());
        if (!l){
            std::cout << "joint named [" << i_names[i] << "] not found"
                      << std::endl;
            ret = false;
        }else{
            o_ids.push_back(l->jointId);
        }
    }
    return ret;
}

void BodyRTC::getStatus(OpenHRP::RobotHardwareService::RobotState* rs) {
    rs->angle.length(numJoints());
    rs->command.length(numJoints());
    for(size_t i = 0; i < numJoints(); i++) {
        rs->angle[i] = angles[i];
        rs->command[i] = commands[i];
    }
    rs->force.length(forces.size());
    for(size_t j = 0; j < forces.size(); j++) {
        rs->force[j].length(6);
        for(size_t i = 0; i < 6; i++ ) rs->force[j][i] = forces[j][i];
    }
    rs->rateGyro.length(gyros.size());
    for(size_t j = 0; j < gyros.size() ; j++) {
        rs->rateGyro[j].length(3);
        for(size_t i = 0; i < 3; i++ ) rs->rateGyro[j][i] = gyros[j][i];
    }
    rs->accel.length(accels.size());
    for(size_t j = 0; j < accels.size(); j++) {
        rs->accel[j].length(3);
        for(size_t i = 0; i < 3; i++ ) rs->accel[j][i] = accels[j][i];
    }

    rs->servoState.length(numJoints());
    int v, status;
    for(unsigned int i=0; i < rs->servoState.length(); ++i){
        //size_t len = lengthOfExtraServoState(i)+1;
        rs->servoState[i].length(1);
        status = 0;
        v = readCalibState(i);
        status |= v<< OpenHRP::RobotHardwareService::CALIB_STATE_SHIFT;
        v = readPowerState(i);
        status |= v<< OpenHRP::RobotHardwareService::POWER_STATE_SHIFT;
        v = readServoState(i);
        status |= v<< OpenHRP::RobotHardwareService::SERVO_STATE_SHIFT;
        //v = readServoAlarm(i);
        //status |= v<< OpenHRP::RobotHardwareService::SERVO_ALARM_SHIFT;
        //v = readDriverTemperature(i);
        //status |= v<< OpenHRP::RobotHardwareService::DRIVER_TEMP_SHIFT;
        rs->servoState[i][0] = status;
        //readExtraServoState(i, (int *)(rs->servoState[i].get_buffer()+1));
    }
    //readPowerStatus(rs->voltage, rs->current);
}

void BodyRTC::getStatus2(OpenHRP::RobotHardwareService::RobotState2* rs) {
}

bool BodyRTC::setServoErrorLimit(const char *i_jname, double i_limit)
{
    Link *l = NULL;
    if (strcmp(i_jname, "all") == 0 || strcmp(i_jname, "ALL") == 0){
        for (unsigned int i=0; i<numJoints(); i++){
            m_servoErrorLimit[i] = i_limit;
        }
    }else if ((l = link(i_jname))){
        m_servoErrorLimit[l->jointId] = i_limit;
    }else{
        char *s = (char *)i_jname; while(*s) {*s=toupper(*s);s++;}
        const std::vector<int> jgroup = m_jointGroups[i_jname];
        if (jgroup.size()==0) return false;
        for (unsigned int i=0; i<jgroup.size(); i++){
            m_servoErrorLimit[jgroup[i]] = i_limit;
        }
    }
    return true;
}

char *time_string()
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    struct tm *tm_ = localtime(&tv.tv_sec);
    static char time[20];
    sprintf(time, "%02d:%02d:%02d.%06d", tm_->tm_hour, tm_->tm_min, tm_->tm_sec, (int)tv.tv_usec);
    return time;
}

#define ON		1
#define OFF		0
bool BodyRTC::checkEmergency(emg_reason &o_reason, int &o_id) {
    int state;

    o_reason = EMG_NONE; // clear
    o_id = -1;

    for (unsigned int i=0; i<numJoints(); i++){
        state = readServoState(i);
        if (state == ON && m_servoErrorLimit[i] != 0){
            double angle, command;
            angle = angles[i];
            command = commands[i];
            if (fabs(angle-command) > m_servoErrorLimit[i]){
                std::cerr << time_string()
                          << ": servo error limit over: joint = "
		          << joint(i)->name
		          << ", qRef = " << command/M_PI*180 << "[deg], q = "
		          << angle/M_PI*180 << "[deg]" << std::endl;
                o_reason = EMG_SERVO_ERROR;
                o_id = i;
                return true;
            }
        }
    }
    return false;
}

bool BodyRTC::preOneStep() {
    // Simulate servo off in HighGain mode simulation
    hrp::Vector3 g(0, 0, 9.8);
    calcCM();
    rootLink()->calcSubMassCM();
    bool all_servo_off = true;
    bool emulate_highgain_servo_off_mode = (numJoints() > 0); // If no joints, do not use servo off emulation
    for(unsigned int i = 0; i < numJoints(); ++i){
        Link *j = joint(i);
        commands[i] = j->q;
        int p = readPowerState(i);
        int s = readServoState(i);
        if ( p && s ) { all_servo_off = false; continue; }
        switch(j->jointType){
            case Link::ROTATIONAL_JOINT:
                {
                    j->q += (j->subm*g).cross(j->submwc / j->subm - j->p).dot(j->R * j->a) *0.005*0.01;
                    if ( j->q < j->llimit ) {
                        j->q = j->llimit;
                    }else if ( j->q > j->ulimit ) {
                        j->q = j->ulimit;
                    }
                }
                break;

            default:
                std::cerr << "calcCMJacobian() : unsupported jointType("
                          << j->jointType << std::endl;
        }
    }
    if ( m_resetPosition ) {
        getDefaultRootPosition(m_lastServoOn_p, m_lastServoOn_R);
        rootLink()->p = m_lastServoOn_p;
        rootLink()->setAttitude(m_lastServoOn_R);
        m_resetPosition = false;
    }
    if (emulate_highgain_servo_off_mode) {
        if ( all_servo_off ) { // when all servo is off, do not move root joint
            rootLink()->p = m_lastServoOn_p;
            rootLink()->setAttitude(m_lastServoOn_R);
        } else {
            m_lastServoOn_p = rootLink()->p;
            m_lastServoOn_R = rootLink()->attitude();
        }
    }
}

bool BodyRTC::postOneStep() {

    for(unsigned int i = 0; i < numJoints(); ++i){
        angles[i] = joint(i)->q;
    }
    for(unsigned int i = 0; i < numSensors(hrp::Sensor::ACCELERATION); i++ ){
        hrp::AccelSensor *s = sensor<AccelSensor>(i);
        accels[i][0] =  s->dv[0];
        accels[i][1] =  s->dv[1];
        accels[i][2] =  s->dv[2];
    }
    for(unsigned int i = 0; i < numSensors(hrp::Sensor::RATE_GYRO); i++ ){
        hrp::RateGyroSensor *s = sensor<RateGyroSensor>(i);
        gyros[i][0] =  s->w[0];
        gyros[i][1] =  s->w[1];
        gyros[i][2] =  s->w[2];
    }
    for(unsigned int i = 0; i < numSensors(hrp::Sensor::FORCE); i++ ){
        hrp::ForceSensor *s = sensor<ForceSensor>(i);
        forces[i][0] =  s->f[0];
        forces[i][1] =  s->f[1];
        forces[i][2] =  s->f[2];
        forces[i][3] =  s->tau[0];
        forces[i][4] =  s->tau[1];
        forces[i][5] =  s->tau[2];
    }
    if ( checkEmergency(m_emergencyReason, m_emergencyId) ) {
        servo("all", false);
    }
    return true;
}

// lib/io/iob.h:
#define JID_ALL -1
#define JID_INVALID -2
// these code are copied from rtc/RobotHardware/robot.cpp
bool BodyRTC::servo(const char *jname, bool turnon)
{
    Link *l = NULL;
    if (strcmp(jname, "all") == 0 || strcmp(jname, "ALL") == 0){
        bool ret = true;
        for (unsigned int i=0; i<numJoints(); i++){
            ret = ret && servo(i, turnon);
        }
        return ret;
    }else if ((l = link(jname))){
        return servo(l->jointId, turnon);
    }else{
        char *s = (char *)jname; while(*s) {*s=toupper(*s);s++;}
        const std::vector<int> jgroup = m_jointGroups[jname];
        if (jgroup.size() == 0) return false;
        bool ret = true;
        for (unsigned int i=0; i<jgroup.size(); i++){
            ret = ret && servo(jgroup[i], turnon);
            return ret;
        }
    }
    return false;
}

bool BodyRTC::power(const char *jname, bool turnon)
{
    int jid = JID_INVALID;

    if (strcmp(jname, "all") == 0 || strcmp(jname, "ALL") == 0){
        jid = JID_ALL;
    }else{
        Link *l = link(jname);
        if (!l) return false;
        jid = l->jointId;
    }
    return power(jid, turnon);
}

RobotHardwareServicePort::RobotHardwareServicePort() {
}
RobotHardwareServicePort::~RobotHardwareServicePort() {
}

void RobotHardwareServicePort::getStatus(OpenHRP::RobotHardwareService::RobotState_out rs) {
    rs = new OpenHRP::RobotHardwareService::RobotState();
    m_robot->getStatus(rs);
}

void RobotHardwareServicePort::getStatus2(OpenHRP::RobotHardwareService::RobotState2_out rs) {
    rs = new OpenHRP::RobotHardwareService::RobotState2();
    m_robot->getStatus2(rs);
}

CORBA::Boolean RobotHardwareServicePort::power(const char* jname, OpenHRP::RobotHardwareService::SwitchStatus turnon) {
    m_robot->power(jname, turnon == OpenHRP::RobotHardwareService::SWITCH_ON);
}

CORBA::Boolean RobotHardwareServicePort::servo(const char* jname, OpenHRP::RobotHardwareService::SwitchStatus turnon) {
    m_robot->servo(jname, turnon == OpenHRP::RobotHardwareService::SWITCH_ON);
}
void RobotHardwareServicePort::setServoGainPercentage(const char *jname, double limit) {
}
void RobotHardwareServicePort::setServoErrorLimit(const char *jname, double limit) {
    m_robot->setServoErrorLimit(jname, limit);
}
void RobotHardwareServicePort::calibrateInertiaSensor() {
}
void RobotHardwareServicePort::removeForceSensorOffset() {
}
void RobotHardwareServicePort::initializeJointAngle(const char* name, const char* option) {
}
CORBA::Boolean RobotHardwareServicePort::addJointGroup(const char* gname, const OpenHRP::RobotHardwareService::StrSequence& jnames) {
    char *s = (char *)gname; while(*s) {*s=toupper(*s);s++;}
    std::vector<int> jids;
    std::vector<std::string> joints;
    joints.resize(jnames.length());
    for (unsigned int i=0; i<jnames.length(); i++){
        joints[i] = jnames[i];
    }
    bool ret = m_robot->names2ids(joints, jids);
    //m_jointGroups[gname] = jids;
    m_robot->addJointGroup(gname, jids);
    return ret;
}
CORBA::Boolean RobotHardwareServicePort::readDigitalInput(::OpenHRP::RobotHardwareService::OctSequence_out din) {
    return false;
}
CORBA::Long RobotHardwareServicePort::lengthDigitalInput() {
    return 0;
}
CORBA::Boolean RobotHardwareServicePort::writeDigitalOutput(const ::OpenHRP::RobotHardwareService::OctSequence& dout) {
    return false;
}
CORBA::Boolean RobotHardwareServicePort::writeDigitalOutputWithMask(const ::OpenHRP::RobotHardwareService::OctSequence& dout, const ::OpenHRP::RobotHardwareService::OctSequence& mask) {
    return false;
}
CORBA::Long RobotHardwareServicePort::lengthDigitalOutput() {
    return 0;
}
CORBA::Boolean RobotHardwareServicePort::readDigitalOutput(::OpenHRP::RobotHardwareService::OctSequence_out dout) {
    return false;
}
//
void RobotHardwareServicePort::setRobot(BodyRTC *i_robot) { m_robot = i_robot; }

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
