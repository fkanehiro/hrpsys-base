#include "CollisionDetectorService_impl.h"
#include "CollisionDetector.h"

CollisionDetectorService_impl::CollisionDetectorService_impl() : m_collision(NULL)
{
}

CollisionDetectorService_impl::~CollisionDetectorService_impl()
{
}

CORBA::Boolean CollisionDetectorService_impl::setTolerance(const char *i_link_pair_name, CORBA::Double d_tolerance)
{
    return m_collision->setTolerance(i_link_pair_name, d_tolerance);
}

void CollisionDetectorService_impl::collision(CollisionDetector *i_collision)
{
    m_collision = i_collision;
} 

