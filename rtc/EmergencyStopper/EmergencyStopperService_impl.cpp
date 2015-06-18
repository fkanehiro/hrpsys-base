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

void EmergencyStopperService_impl::emergencystopper(EmergencyStopper *i_emergencystopper)
{
    m_emergencystopper = i_emergencystopper;
}
