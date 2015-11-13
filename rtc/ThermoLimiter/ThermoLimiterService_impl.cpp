// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 4; -*-
#include <iostream>
#include "ThermoLimiterService_impl.h"
#include "ThermoLimiter.h"

ThermoLimiterService_impl::ThermoLimiterService_impl() : m_thermolimiter(NULL)
{
}

ThermoLimiterService_impl::~ThermoLimiterService_impl()
{
}

CORBA::Boolean ThermoLimiterService_impl::setParameter(const OpenHRP::ThermoLimiterService::tlParam& i_param)
{
    return m_thermolimiter->setParameter(i_param);
};

CORBA::Boolean ThermoLimiterService_impl::getParameter(OpenHRP::ThermoLimiterService::tlParam_out i_param)
{
    i_param = OpenHRP::ThermoLimiterService::tlParam();
    return m_thermolimiter->getParameter(i_param);
};

void ThermoLimiterService_impl::thermolimiter(ThermoLimiter *i_thermolimiter)
{
    m_thermolimiter = i_thermolimiter;
}
