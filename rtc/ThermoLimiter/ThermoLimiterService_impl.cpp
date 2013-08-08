// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 4; -*-
#include <iostream>
#include "NullService_impl.h"

NullService_impl::NullService_impl()
{
}

NullService_impl::~NullService_impl()
{
}

void NullService_impl::echo(const char *msg)
{
	std::cout << "NullService: " << msg << std::endl;
}

