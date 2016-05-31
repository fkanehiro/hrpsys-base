// -*- C++ -*-
#include <iostream>
#include "ReferenceForceUpdaterService_impl.h"
#include "ReferenceForceUpdater.h"

ReferenceForceUpdaterService_impl::ReferenceForceUpdaterService_impl()
{
}

ReferenceForceUpdaterService_impl::~ReferenceForceUpdaterService_impl()
{
}

CORBA::Boolean ReferenceForceUpdaterService_impl::setReferenceForceUpdaterParam(const char *i_name_, const OpenHRP::ReferenceForceUpdaterService::ReferenceForceUpdaterParam& i_param)
{
    return m_rfu->setReferenceForceUpdaterParam(std::string(i_name_), i_param);
};

CORBA::Boolean ReferenceForceUpdaterService_impl::getReferenceForceUpdaterParam(const char *i_name_, OpenHRP::ReferenceForceUpdaterService::ReferenceForceUpdaterParam_out i_param)
{
    i_param = new OpenHRP::ReferenceForceUpdaterService::ReferenceForceUpdaterParam();
    i_param->motion_dir.length(3);
    return m_rfu->getReferenceForceUpdaterParam(std::string(i_name_), i_param);
};

CORBA::Boolean ReferenceForceUpdaterService_impl::startReferenceForceUpdater(const char *i_name_)
{
    return m_rfu->startReferenceForceUpdater(std::string(i_name_));
};

CORBA::Boolean ReferenceForceUpdaterService_impl::stopReferenceForceUpdater(const char *i_name_)
{
    return m_rfu->stopReferenceForceUpdater(std::string(i_name_));
};

void ReferenceForceUpdaterService_impl::rfu(ReferenceForceUpdater *i_rfu)
{
    m_rfu = i_rfu;
};

