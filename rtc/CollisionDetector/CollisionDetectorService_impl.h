// -*-C++-*-
#ifndef COLLISIONDETECTORSERVICE_IMPL_H
#define COLLISIONDETECTORSERVICE_IMPL_H

#include "hrpsys/idl/CollisionDetectorService.hh"

using namespace OpenHRP;

class CollisionDetector;

class CollisionDetectorService_impl 
    : public virtual POA_OpenHRP::CollisionDetectorService,
      public virtual PortableServer::RefCountServantBase
{
public:
    CollisionDetectorService_impl();
    virtual ~CollisionDetectorService_impl();
    //
    CORBA::Boolean enableCollisionDetection();
    CORBA::Boolean disableCollisionDetection();
    CORBA::Boolean setTolerance(const char *i_link_pair_name, CORBA::Double d_tolerance);
    CORBA::Boolean setCollisionLoop(CORBA::Short loop);
    CORBA::Boolean getCollisionStatus(OpenHRP::CollisionDetectorService::CollisionState_out state);
    void collision(CollisionDetector *i_collision);
    //
private:
    CollisionDetector *m_collision;
};				 

#endif
