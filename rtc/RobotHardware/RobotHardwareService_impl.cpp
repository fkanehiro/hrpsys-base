#include "RobotHardwareService_impl.h"
#include "robot.h"
#include <hrpModel/Sensor.h>

using namespace OpenHRP;
using namespace hrp;

RobotHardwareService_impl::RobotHardwareService_impl() : m_robot(boost::shared_ptr<robot>()) 
{
}

RobotHardwareService_impl::~RobotHardwareService_impl() 
{
}

#define GetStatus                                                       \
                                                                        \
    rs->angle.length(m_robot->numJoints());                             \
    m_robot->readJointAngles(rs->angle.get_buffer());                   \
                                                                        \
    rs->command.length(m_robot->numJoints());                           \
    m_robot->readJointCommands(rs->command.get_buffer());               \
                                                                        \
    rs->torque.length(m_robot->numJoints());                            \
    if (!m_robot->readJointTorques(rs->torque.get_buffer())){           \
        for (unsigned int i=0; i<rs->torque.length(); i++){             \
            rs->torque[i] = 0.0;                                        \
        }                                                               \
    }                                                                   \
                                                                        \
    rs->servoState.length(m_robot->numJoints());                        \
    int v, status;                                                      \
    for(unsigned int i=0; i < rs->servoState.length(); ++i){            \
        size_t len = m_robot->lengthOfExtraServoState(i)+1;             \
        rs->servoState[i].length(len);                                  \
	status = 0;                                                     \
        v = m_robot->readCalibState(i);                                 \
        status |= v<< OpenHRP::RobotHardwareService::CALIB_STATE_SHIFT; \
        v = m_robot->readPowerState(i);                                 \
        status |= v<< OpenHRP::RobotHardwareService::POWER_STATE_SHIFT; \
        v = m_robot->readServoState(i);                                 \
        status |= v<< OpenHRP::RobotHardwareService::SERVO_STATE_SHIFT; \
        v = m_robot->readServoAlarm(i);                                 \
        status |= v<< OpenHRP::RobotHardwareService::SERVO_ALARM_SHIFT; \
        v = m_robot->readDriverTemperature(i);                          \
        status |= v<< OpenHRP::RobotHardwareService::DRIVER_TEMP_SHIFT; \
        rs->servoState[i][0] = status;                                  \
        m_robot->readExtraServoState(i, (int *)(rs->servoState[i].get_buffer()+1)); \
    }                                                                   \
                                                                        \
    rs->rateGyro.length(m_robot->numSensors(Sensor::RATE_GYRO));        \
    for (unsigned int i=0; i<rs->rateGyro.length(); i++){               \
        rs->rateGyro[i].length(3);                                      \
        m_robot->readGyroSensor(i, rs->rateGyro[i].get_buffer());       \
    }                                                                   \
                                                                        \
    rs->accel.length(m_robot->numSensors(Sensor::ACCELERATION));        \
    for (unsigned int i=0; i<rs->accel.length(); i++){                  \
        rs->accel[i].length(3);                                         \
        m_robot->readAccelerometer(i, rs->accel[i].get_buffer());       \
    }                                                                   \
                                                                        \
    rs->force.length(m_robot->numSensors(Sensor::FORCE));               \
    for (unsigned int i=0; i<rs->force.length(); i++){                  \
        rs->force[i].length(6);                                         \
        m_robot->readForceSensor(i, rs->force[i].get_buffer());         \
    }									\
									\
    m_robot->readPowerStatus(rs->voltage, rs->current);

void RobotHardwareService_impl::getStatus(OpenHRP::RobotHardwareService::RobotState_out rs)
{
    rs = new OpenHRP::RobotHardwareService::RobotState();

    GetStatus;
}

void RobotHardwareService_impl::getStatus2(OpenHRP::RobotHardwareService::RobotState2_out rs)
{
    rs = new OpenHRP::RobotHardwareService::RobotState2();

    GetStatus;

#if defined(ROBOT_IOB_VERSION) && ROBOT_IOB_VERSION >= 2
    rs->batteries.length(m_robot->numBatteries());
    for(unsigned int i=0; i<rs->batteries.length(); i++){
        m_robot->readBatteryState(i, 
                                  rs->batteries[i].voltage,
                                  rs->batteries[i].current,
                                  rs->batteries[i].soc);
    }
    rs->temperature.length(m_robot->numThermometers());
    for (unsigned int i=0; i<rs->temperature.length(); i++){
        m_robot->readThermometer(i, rs->temperature[i]);
    }
#endif
}

CORBA::Boolean RobotHardwareService_impl::power(const char* jname, OpenHRP::RobotHardwareService::SwitchStatus ss)
{
    return m_robot->power(jname, ss==OpenHRP::RobotHardwareService::SWITCH_ON);
}

CORBA::Boolean RobotHardwareService_impl::servo(const char* jname, OpenHRP::RobotHardwareService::SwitchStatus ss)
{
    return m_robot->servo(jname, ss==OpenHRP::RobotHardwareService::SWITCH_ON);
}

void RobotHardwareService_impl::calibrateInertiaSensor()
{
    m_robot->startInertiaSensorCalibration();
}

void RobotHardwareService_impl::removeForceSensorOffset()
{
    m_robot->removeForceSensorOffset();
}

void RobotHardwareService_impl::initializeJointAngle(const char* name, const char* option)
{
    m_robot->initializeJointAngle(name, option);
}

void RobotHardwareService_impl::setServoGainPercentage(const char *jname, double percentage)
{
    m_robot->setServoGainPercentage(jname, percentage);
}

void RobotHardwareService_impl::setServoTorqueGainPercentage(const char *jname, double percentage)
{
    m_robot->setServoTorqueGainPercentage(jname, percentage);
}

void RobotHardwareService_impl::setServoErrorLimit(const char *jname, double limit)
{
    m_robot->setServoErrorLimit(jname, limit);
}

CORBA::Boolean RobotHardwareService_impl::addJointGroup(const char* gname, const OpenHRP::RobotHardwareService::StrSequence& jnames)
{
    std::vector<std::string> joints;
    joints.resize(jnames.length());
    for (unsigned int i=0; i<jnames.length(); i++){
        joints[i] = jnames[i];
    }
    return m_robot->addJointGroup(gname, joints);
}

CORBA::Boolean RobotHardwareService_impl::readDigitalInput(::OpenHRP::RobotHardwareService::OctSequence_out din)
{
    din = new ::OpenHRP::RobotHardwareService::OctSequence();
    din->length(lengthDigitalInput());
    return m_robot->readDigitalInput((char *)(din->get_buffer()));
}

CORBA::Long RobotHardwareService_impl::lengthDigitalInput()
{
    return m_robot->lengthDigitalInput();
}

CORBA::Boolean RobotHardwareService_impl::writeDigitalOutput(const ::OpenHRP::RobotHardwareService::OctSequence& dout)
{
    return m_robot->writeDigitalOutput((const char *)(dout.get_buffer()));
}

CORBA::Boolean RobotHardwareService_impl::writeDigitalOutputWithMask(const ::OpenHRP::RobotHardwareService::OctSequence& dout, const ::OpenHRP::RobotHardwareService::OctSequence& mask)
{
    return m_robot->writeDigitalOutputWithMask((const char *)(dout.get_buffer()), (const char *)(mask.get_buffer()));
}

CORBA::Long RobotHardwareService_impl::lengthDigitalOutput()
{
    return m_robot->lengthDigitalOutput();
}

CORBA::Boolean RobotHardwareService_impl::readDigitalOutput(::OpenHRP::RobotHardwareService::OctSequence_out dout)
{
    dout = new ::OpenHRP::RobotHardwareService::OctSequence();
    dout->length(lengthDigitalOutput());
    return m_robot->readDigitalOutput((char *)(dout->get_buffer()));
}

CORBA::Boolean RobotHardwareService_impl::setJointInertia(const char* name, ::CORBA::Double mn)
{
    m_robot->setJointInertia(name, mn);
}

void RobotHardwareService_impl::setJointInertias(const ::OpenHRP::RobotHardwareService::DblSequence& mns)
{
    m_robot->setJointInertias(mns.get_buffer());
}


void RobotHardwareService_impl::enableDisturbanceObserver()
{
    m_robot->enableDisturbanceObserver();
}

void RobotHardwareService_impl::disableDisturbanceObserver()
{
    m_robot->disableDisturbanceObserver();
}

void RobotHardwareService_impl::setDisturbanceObserverGain(::CORBA::Double gain)
{
    m_robot->setDisturbanceObserverGain(gain);
}

void RobotHardwareService_impl::setJointControlMode(const char *jname, OpenHRP::RobotHardwareService::JointControlMode jcm)
{
    joint_control_mode mode;
    switch(jcm){
    case OpenHRP::RobotHardwareService::FREE:
        mode = JCM_FREE;
        break;
    case OpenHRP::RobotHardwareService::POSITION:
        mode = JCM_POSITION;
        break;
    case OpenHRP::RobotHardwareService::TORQUE:
        mode = JCM_TORQUE;
        break;
    case OpenHRP::RobotHardwareService::VELOCITY:
        mode = JCM_VELOCITY;
        break;
#if defined(ROBOT_IOB_VERSION) && ROBOT_IOB_VERSION >= 4
    case OpenHRP::RobotHardwareService::POSITION_TORQUE:
        mode = JCM_POSITION_TORQUE;
        break;
#endif
    default:
        return;
    }
    m_robot->setJointControlMode(jname, mode);
}
