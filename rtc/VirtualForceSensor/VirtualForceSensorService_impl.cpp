// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 4; -*-
#include <iostream>
#include "VirtualForceSensorService_impl.h"
#include "VirtualForceSensor.h"

VirtualForceSensorService_impl::VirtualForceSensorService_impl() : m_vfsensor(NULL)
{
}

VirtualForceSensorService_impl::~VirtualForceSensorService_impl()
{
}

CORBA::Boolean VirtualForceSensorService_impl::removeVirtualForceSensorOffset(const char *sensorName)
{
	return m_vfsensor->removeVirtualForceSensorOffset(std::string(sensorName));
}

void VirtualForceSensorService_impl::vfsensor(VirtualForceSensor *i_vfsensor)
{
	m_vfsensor = i_vfsensor;
}

