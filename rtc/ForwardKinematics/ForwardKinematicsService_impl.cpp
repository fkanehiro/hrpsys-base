#include "ForwardKinematicsService_impl.h"
#include "ForwardKinematics.h"

ForwardKinematicsService_impl::ForwardKinematicsService_impl() : m_comp(NULL)
{
}

ForwardKinematicsService_impl::~ForwardKinematicsService_impl()
{
}


void ForwardKinematicsService_impl::setComp(ForwardKinematics *i_comp)
{
    m_comp = i_comp;
}

::CORBA::Boolean ForwardKinematicsService_impl::selectBaseLink(const char* linkname)
{
    return m_comp->selectBaseLink(linkname);
}


::CORBA::Boolean ForwardKinematicsService_impl::getReferencePose(const char* linkname, RTC::TimedDoubleSeq_out pose)
{
    return m_comp->getReferencePose(linkname, pose);
}
::CORBA::Boolean ForwardKinematicsService_impl::getCurrentPose(const char* linkname, RTC::TimedDoubleSeq_out pose)
{
    return m_comp->getCurrentPose(linkname, pose);
}

::CORBA::Boolean ForwardKinematicsService_impl::getRelativeCurrentPosition(const char* linknameFrom, const char *linknameTo, const OpenHRP::ForwardKinematicsService::position target, OpenHRP::ForwardKinematicsService::position result)
{
    return m_comp->getRelativeCurrentPosition(linknameFrom, linknameTo, target, result);
}
