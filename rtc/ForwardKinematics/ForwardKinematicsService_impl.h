#ifndef __FORWARD_KINEMATICS_SERVICE_IMPL_H__
#define __FORWARD_KINEMATICS_SERVICE_IMPL_H__

#include "ForwardKinematicsService.hh"

class ForwardKinematics;

class ForwardKinematicsService_impl
    : public virtual POA_OpenHRP::ForwardKinematicsService,
      public virtual PortableServer::RefCountServantBase
{
public:
    /**
       \brief constructor
    */
    ForwardKinematicsService_impl();
    
    /**
       \brief destructor
    */
    virtual ~ForwardKinematicsService_impl();

    void setComp(ForwardKinematics *i_comp);
    //
    ::CORBA::Boolean selectBaseLink(const char* lnkname);
    ::CORBA::Boolean getReferencePose(const char* linkname, RTC::TimedDoubleSeq_out pose);
    ::CORBA::Boolean getCurrentPose(const char* linkname, RTC::TimedDoubleSeq_out pose);
    ::CORBA::Boolean getRelativeCurrentPosition(const char* linkname1, const char *linkname2, const OpenHRP::ForwardKinematicsService::position target, OpenHRP::ForwardKinematicsService::position result);
private:
    ForwardKinematics *m_comp;
};
    
#endif
