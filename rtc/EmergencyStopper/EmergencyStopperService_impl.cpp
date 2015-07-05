#include <iostream>
#include "EmergencyStopperService_impl.h"
#include "EmergencyStopper.h"

EmergencyStopperService_impl::EmergencyStopperService_impl()
{
}

EmergencyStopperService_impl::~EmergencyStopperService_impl()
{
}

void EmergencyStopperService_impl::stopMotion()
{
    m_emergencystopper->stopMotion();
}

void EmergencyStopperService_impl::releaseMotion()
{
    m_emergencystopper->releaseMotion();
}

CORBA::Boolean EmergencyStopperService_impl::getEmergencyStopperParam(OpenHRP::EmergencyStopperService::EmergencyStopperParam& i_param)
{
    return m_emergencystopper->getEmergencyStopperParam(i_param);
};

CORBA::Boolean EmergencyStopperService_impl::setEmergencyStopperParam(const OpenHRP::EmergencyStopperService::EmergencyStopperParam& i_param)
{
    return m_emergencystopper->setEmergencyStopperParam(i_param);
};

void EmergencyStopperService_impl::emergencystopper(EmergencyStopper *i_emergencystopper)
{
    m_emergencystopper = i_emergencystopper;
}
