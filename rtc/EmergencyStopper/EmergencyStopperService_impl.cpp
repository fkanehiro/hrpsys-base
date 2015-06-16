// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 4; -*-
#include <iostream>
#include "EmergencyStopperService_impl.h"

EmergencyStopperService_impl::EmergencyStopperService_impl()
{
}

EmergencyStopperService_impl::~EmergencyStopperService_impl()
{
}

void EmergencyStopperService_impl::echo(const char *msg)
{
	std::cout << "EmergencyStopperService: " << msg << std::endl;
}

