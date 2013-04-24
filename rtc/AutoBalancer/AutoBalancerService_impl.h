// -*-C++-*-
#ifndef AUTOBALANCERSERVICESVC_IMPL_H
#define AUTOBALANCERSERVICESVC_IMPL_H

#include "AutoBalancerService.hh"

using namespace OpenHRP;

class AutoBalancer;

class AutoBalancerService_impl 
  : public virtual POA_OpenHRP::AutoBalancerService,
    public virtual PortableServer::RefCountServantBase
{
public:
  AutoBalancerService_impl();
  virtual ~AutoBalancerService_impl();
  CORBA::Boolean goPos( CORBA::Double x,  CORBA::Double y,  CORBA::Double th);
  CORBA::Boolean goVelocity( CORBA::Double vx,  CORBA::Double vy,  CORBA::Double vth);
  CORBA::Boolean goStop();
  CORBA::Boolean setFootSteps(const OpenHRP::AutoBalancerService::FootstepSequence& fs);
  void waitFootSteps();
  CORBA::Boolean startABC(const OpenHRP::AutoBalancerService::AutoBalancerLimbParamSequence& alp);
  CORBA::Boolean stopABC();
  //
  //
  void autobalancer(AutoBalancer *i_autobalancer);
private:
  AutoBalancer *m_autobalancer;
};				 

#endif
