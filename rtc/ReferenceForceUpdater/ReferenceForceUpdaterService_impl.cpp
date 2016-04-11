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

CORBA::Boolean ReferenceForceUpdaterService_impl::setReferenceForceUpdaterParam(const OpenHRP::ReferenceForceUpdaterService::ReferenceForceUpdaterParam& i_param)
{
    return m_rfu->setReferenceForceUpdaterParam(i_param);
};

CORBA::Boolean ReferenceForceUpdaterService_impl::getReferenceForceUpdaterParam(OpenHRP::ReferenceForceUpdaterService::ReferenceForceUpdaterParam_out i_param)
{
    i_param = new OpenHRP::ReferenceForceUpdaterService::ReferenceForceUpdaterParam();
    i_param->motion_dir.length(3);
    return m_rfu->getReferenceForceUpdaterParam(i_param);
};

CORBA::Boolean ReferenceForceUpdaterService_impl::startReferenceForceUpdater()
{
    return m_rfu->startReferenceForceUpdater();
};

CORBA::Boolean ReferenceForceUpdaterService_impl::stopReferenceForceUpdater()
{
    return m_rfu->stopReferenceForceUpdater();
};

void ReferenceForceUpdaterService_impl::rfu(ReferenceForceUpdater *i_rfu)
{
    m_rfu = i_rfu;
};

