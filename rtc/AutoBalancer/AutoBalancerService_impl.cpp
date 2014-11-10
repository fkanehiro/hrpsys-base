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

CORBA::Boolean AutoBalancerService_impl::startAutoBalancer(const OpenHRP::AutoBalancerService::StrSequence& limbs)
{
  return m_autobalancer->startAutoBalancer(limbs);
};

CORBA::Boolean AutoBalancerService_impl::stopAutoBalancer()
{
  return m_autobalancer->stopAutoBalancer();
};

CORBA::Boolean AutoBalancerService_impl::setGaitGeneratorParam(const OpenHRP::AutoBalancerService::GaitGeneratorParam& i_param)
{
  return m_autobalancer->setGaitGeneratorParam(i_param);
};

CORBA::Boolean AutoBalancerService_impl::getGaitGeneratorParam(OpenHRP::AutoBalancerService::GaitGeneratorParam_out i_param)
{
  i_param = new OpenHRP::AutoBalancerService::GaitGeneratorParam();
  i_param->stride_parameter.length(4);
  return m_autobalancer->getGaitGeneratorParam(*i_param);
};

CORBA::Boolean AutoBalancerService_impl::setAutoBalancerParam(const OpenHRP::AutoBalancerService::AutoBalancerParam& i_param)
{
  return m_autobalancer->setAutoBalancerParam(i_param);
};

CORBA::Boolean AutoBalancerService_impl::getAutoBalancerParam(OpenHRP::AutoBalancerService::AutoBalancerParam_out i_param)
{
  i_param = new OpenHRP::AutoBalancerService::AutoBalancerParam();
  i_param->default_zmp_offsets.length(2);
  for (size_t i = 0; i < 2; i++)
    i_param->default_zmp_offsets[i].length(3);
  return m_autobalancer->getAutoBalancerParam(*i_param);
};

CORBA::Boolean AutoBalancerService_impl::getFootstepParam(OpenHRP::AutoBalancerService::FootstepParam_out i_param)
{
  i_param = new OpenHRP::AutoBalancerService::FootstepParam();
  return m_autobalancer->getFootstepParam(*i_param);
};

void AutoBalancerService_impl::autobalancer(AutoBalancer *i_autobalancer)
{
  m_autobalancer = i_autobalancer;
} 

