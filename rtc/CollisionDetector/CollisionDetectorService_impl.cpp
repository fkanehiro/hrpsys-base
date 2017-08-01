#include "CollisionDetectorService_impl.h"
#include "CollisionDetector.h"

CollisionDetectorService_impl::CollisionDetectorService_impl() : m_collision(NULL)
{
}

CollisionDetectorService_impl::~CollisionDetectorService_impl()
{
}

CORBA::Boolean CollisionDetectorService_impl::enableCollisionDetection()
{
    return m_collision->enable();
}

CORBA::Boolean CollisionDetectorService_impl::disableCollisionDetection()
{
    return m_collision->disable();
}

CORBA::Boolean CollisionDetectorService_impl::setTolerance(const char *i_link_pair_name, CORBA::Double d_tolerance)
{
    return m_collision->setTolerance(i_link_pair_name, d_tolerance);
}

CORBA::Boolean CollisionDetectorService_impl::setCollisionLoop(CORBA::Short loop)
{
	return m_collision->setCollisionLoop(loop);
}

CORBA::Boolean CollisionDetectorService_impl::getCollisionStatus(OpenHRP::CollisionDetectorService::CollisionState_out state)
{
    state = new OpenHRP::CollisionDetectorService::CollisionState;
    return m_collision->getCollisionStatus(*state);
}

void CollisionDetectorService_impl::collision(CollisionDetector *i_collision)
{
    m_collision = i_collision;
} 

