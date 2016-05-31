// -*- C++ -*-
#ifndef __REFERENCEFORCEUPDATOR_SERVICE_H__
#define __REFERENCEFORCEUPDATOR_SERVICE_H__

#include "hrpsys/idl/ReferenceForceUpdaterService.hh"

using namespace OpenHRP;

class ReferenceForceUpdater;

class ReferenceForceUpdaterService_impl
    : public virtual POA_OpenHRP::ReferenceForceUpdaterService,
      public virtual PortableServer::RefCountServantBase
{
public:
    /**
       \brief constructor
    */
    ReferenceForceUpdaterService_impl();

    /**
       \brief destructor
    */
    virtual ~ReferenceForceUpdaterService_impl();

    CORBA::Boolean setReferenceForceUpdaterParam(const char *i_name_, const OpenHRP::ReferenceForceUpdaterService::ReferenceForceUpdaterParam& i_param);
    CORBA::Boolean getReferenceForceUpdaterParam(const char *i_name_, OpenHRP::ReferenceForceUpdaterService::ReferenceForceUpdaterParam_out i_param);
    CORBA::Boolean startReferenceForceUpdater(const char *i_name_);
    CORBA::Boolean stopReferenceForceUpdater(const char *i_name_);

    void rfu(ReferenceForceUpdater *i_rfu);
private:
    ReferenceForceUpdater* m_rfu;
};

#endif
