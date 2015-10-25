// -*- coding:utf-8-unix; mode: c++; indent-tabs-mode: nil; c-basic-offset: 2; -*-
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

