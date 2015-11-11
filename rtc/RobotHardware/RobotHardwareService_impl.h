// -*- coding:utf-8-unix; mode: c++; indent-tabs-mode: nil; c-basic-offset: 2; -*-

#ifndef ROBOTHARDWARESERVICE_IMPL_H
#define ROBOTHARDWARESERVICE_IMPL_H

#include <boost/intrusive_ptr.hpp>
#include "RobotHardwareService.hh"

#include "robot.h"

class RobotHardwareService_impl
    : public virtual POA_OpenHRP::RobotHardwareService,
      public virtual PortableServer::RefCountServantBase
{
public:
    RobotHardwareService_impl();
    virtual ~RobotHardwareService_impl();

    void getStatus(OpenHRP::RobotHardwareService::RobotState_out rs);
    void getStatus2(OpenHRP::RobotHardwareService::RobotState2_out rs);

    CORBA::Boolean power(const char* jname, OpenHRP::RobotHardwareService::SwitchStatus ss);
    CORBA::Boolean servo(const char* jname, OpenHRP::RobotHardwareService::SwitchStatus ss);
    void setServoGainPercentage(const char *jname, double limit);
    void setServoErrorLimit(const char *jname, double limit);
    void calibrateInertiaSensor();
    void removeForceSensorOffset();
    void initializeJointAngle(const char* name, const char* option);
    CORBA::Boolean addJointGroup(const char* gname, const OpenHRP::RobotHardwareService::StrSequence& jnames);
    CORBA::Boolean readDigitalInput(::OpenHRP::RobotHardwareService::OctSequence_out din);
    CORBA::Long lengthDigitalInput();
    CORBA::Boolean writeDigitalOutput(const ::OpenHRP::RobotHardwareService::OctSequence& dout);
    CORBA::Boolean writeDigitalOutputWithMask(const ::OpenHRP::RobotHardwareService::OctSequence& dout, const ::OpenHRP::RobotHardwareService::OctSequence& mask);
    CORBA::Long lengthDigitalOutput();
    CORBA::Boolean readDigitalOutput(::OpenHRP::RobotHardwareService::OctSequence_out dout);
    //
    void setRobot(boost::shared_ptr<robot>& i_robot) { m_robot = i_robot; }
private:
    boost::shared_ptr<robot> m_robot;
};
#endif
