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

CORBA::Boolean AutoBalancerService_impl::emergencyStop()
{
  return m_autobalancer->emergencyStop();
};

CORBA::Boolean AutoBalancerService_impl::setFootSteps(const OpenHRP::AutoBalancerService::FootstepsSequence& fss, CORBA::Long overwrite_fs_idx)
{
  return m_autobalancer->setFootSteps(fss, overwrite_fs_idx);
}

CORBA::Boolean AutoBalancerService_impl::setFootStepsWithParam(const OpenHRP::AutoBalancerService::FootstepsSequence& fss, const OpenHRP::AutoBalancerService::StepParamsSequence& spss, CORBA::Long overwrite_fs_idx)
{
  return m_autobalancer->setFootStepsWithParam(fss, spss, overwrite_fs_idx);
}

void AutoBalancerService_impl::waitFootSteps()
{
  return m_autobalancer->waitFootSteps();
};

void AutoBalancerService_impl::waitFootStepsEarly(CORBA::Double tm)
{
  return m_autobalancer->waitFootStepsEarly(tm);
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
  i_param->toe_heel_phase_ratio.length(7);
  i_param->zmp_weight_map.length(4);
  return m_autobalancer->getGaitGeneratorParam(*i_param);
};

CORBA::Boolean AutoBalancerService_impl::setAutoBalancerParam(const OpenHRP::AutoBalancerService::AutoBalancerParam& i_param)
{
  return m_autobalancer->setAutoBalancerParam(i_param);
};

CORBA::Boolean AutoBalancerService_impl::getAutoBalancerParam(OpenHRP::AutoBalancerService::AutoBalancerParam_out i_param)
{
  i_param = new OpenHRP::AutoBalancerService::AutoBalancerParam();
  return m_autobalancer->getAutoBalancerParam(*i_param);
};

CORBA::Boolean AutoBalancerService_impl::getFootstepParam(OpenHRP::AutoBalancerService::FootstepParam_out i_param)
{
  i_param = new OpenHRP::AutoBalancerService::FootstepParam();
  return m_autobalancer->getFootstepParam(*i_param);
};

CORBA::Boolean AutoBalancerService_impl::adjustFootSteps(const OpenHRP::AutoBalancerService::Footstep& rfootstep, const OpenHRP::AutoBalancerService::Footstep& lfootstep)
{
    return m_autobalancer->adjustFootSteps(rfootstep, lfootstep);
};

CORBA::Boolean AutoBalancerService_impl::getRemainingFootstepSequence(OpenHRP::AutoBalancerService::FootstepSequence_out o_footstep, CORBA::Long& o_current_fs_idx)
{
    return m_autobalancer->getRemainingFootstepSequence(o_footstep, o_current_fs_idx);
};

CORBA::Boolean AutoBalancerService_impl::getGoPosFootstepsSequence(CORBA::Double x, CORBA::Double y, CORBA::Double th, OpenHRP::AutoBalancerService::FootstepsSequence_out o_footstep)
{
    return m_autobalancer->getGoPosFootstepsSequence(x, y, th, o_footstep);
};

CORBA::Boolean AutoBalancerService_impl::releaseEmergencyStop()
{
    return m_autobalancer->releaseEmergencyStop();
};

void AutoBalancerService_impl::autobalancer(AutoBalancer *i_autobalancer)
{
  m_autobalancer = i_autobalancer;
} 

