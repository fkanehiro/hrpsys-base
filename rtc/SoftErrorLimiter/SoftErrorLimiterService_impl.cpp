// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 4; -*-
#include <iostream>
#include "SoftErrorLimiterService_impl.h"

SoftErrorLimiterService_impl::SoftErrorLimiterService_impl() : m_robot(boost::shared_ptr<robot>()) 
{
}

SoftErrorLimiterService_impl::~SoftErrorLimiterService_impl()
{
}

void SoftErrorLimiterService_impl::setServoErrorLimit(const char *jname, double limit)
{
    m_robot->setServoErrorLimit(jname, limit);
}

