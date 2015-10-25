// -*- coding:utf-8-unix; mode: c++; indent-tabs-mode: nil; c-basic-offset: 2; -*-
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

