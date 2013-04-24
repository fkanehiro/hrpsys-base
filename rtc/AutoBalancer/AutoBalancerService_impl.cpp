#include "AutoBalancerService_impl.h"
#include "AutoBalancer.h"

AutoBalancerService_impl::AutoBalancerService_impl() : m_autobalancer(NULL)
{
}

AutoBalancerService_impl::~AutoBalancerService_impl()
{
}

CORBA::Boolean AutoBalancerService_impl::goPos( CORBA::Double x,  CORBA::Double y,  CORBA::Double th)
{
  return m_autobalancer->goPos(x, y, th);
};

CORBA::Boolean AutoBalancerService_impl::goVelocity( CORBA::Double vx,  CORBA::Double vy,  CORBA::Double vth)
{
  return m_autobalancer->goVelocity(vx, vy, vth);
};

CORBA::Boolean AutoBalancerService_impl::goStop()
{
  return m_autobalancer->goStop();
};

CORBA::Boolean AutoBalancerService_impl::setFootSteps(const OpenHRP::AutoBalancerService::FootstepSequence& fs)
{
  return m_autobalancer->setFootSteps(fs);
}

void AutoBalancerService_impl::waitFootSteps()
{
  return m_autobalancer->waitFootSteps();
};

CORBA::Boolean AutoBalancerService_impl::startABC(const OpenHRP::AutoBalancerService::AutoBalancerLimbParamSequence& alp)
{
  return m_autobalancer->startABC(alp);
};

CORBA::Boolean AutoBalancerService_impl::stopABC()
{
  return m_autobalancer->stopABC();
};

void AutoBalancerService_impl::autobalancer(AutoBalancer *i_autobalancer)
{
  m_autobalancer = i_autobalancer;
} 

