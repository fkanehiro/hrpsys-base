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
    char* frame_name = (char *)strrchr(linkname, ':');
    if ( frame_name ) {
        ((char *)linkname)[frame_name - linkname] = '\0'; // cut frame_name, linkname[strpos(':')] = 0x00
        frame_name++; // skip ":"
    }
    return m_comp->getReferencePose(linkname, pose, frame_name);
}
::CORBA::Boolean ForwardKinematicsService_impl::getCurrentPose(const char* linkname, RTC::TimedDoubleSeq_out pose)
{
    char* frame_name = (char *)strrchr(linkname, ':');
    if ( frame_name ) {
        ((char *)linkname)[frame_name - linkname] = '\0'; // cut frame_name, linkname[strpos(':')] = 0x00
        frame_name++; // skip ":"
    }
    return m_comp->getCurrentPose(linkname, pose, frame_name);
}

::CORBA::Boolean ForwardKinematicsService_impl::getRelativeCurrentPosition(const char* linknameFrom, const char *linknameTo, const OpenHRP::ForwardKinematicsService::position target, OpenHRP::ForwardKinematicsService::position result)
{
    return m_comp->getRelativeCurrentPosition(linknameFrom, linknameTo, target, result);
}
