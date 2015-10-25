// -*- coding:utf-8-unix; mode: c++; indent-tabs-mode: nil; c-basic-offset: 2; -*-
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

