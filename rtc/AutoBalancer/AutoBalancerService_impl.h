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
  CORBA::Boolean emergencyStop();
  CORBA::Boolean setFootSteps(const OpenHRP::AutoBalancerService::FootstepsSequence& fss, CORBA::Long overwrite_fs_idx);
  CORBA::Boolean setFootStepsWithParam(const OpenHRP::AutoBalancerService::FootstepsSequence& fss, const OpenHRP::AutoBalancerService::StepParamsSequence& spss, CORBA::Long overwrite_fs_idx);
  void waitFootSteps();
  void waitFootStepsEarly(CORBA::Double tm);
  CORBA::Boolean startAutoBalancer(const OpenHRP::AutoBalancerService::StrSequence& limbs);
  CORBA::Boolean stopAutoBalancer();
  CORBA::Boolean setGaitGeneratorParam(const OpenHRP::AutoBalancerService::GaitGeneratorParam& i_param);
  CORBA::Boolean getGaitGeneratorParam(OpenHRP::AutoBalancerService::GaitGeneratorParam_out i_param);
  CORBA::Boolean setAutoBalancerParam(const OpenHRP::AutoBalancerService::AutoBalancerParam& i_param);
  CORBA::Boolean getAutoBalancerParam(OpenHRP::AutoBalancerService::AutoBalancerParam_out i_param);
  CORBA::Boolean getFootstepParam(OpenHRP::AutoBalancerService::FootstepParam_out i_param);
  CORBA::Boolean adjustFootSteps(const OpenHRP::AutoBalancerService::Footstep& rfootstep, const OpenHRP::AutoBalancerService::Footstep& lfootstep);
  CORBA::Boolean getRemainingFootstepSequence(OpenHRP::AutoBalancerService::FootstepSequence_out o_footstep , CORBA::Long& o_current_fs_idx);
  CORBA::Boolean getGoPosFootstepsSequence(CORBA::Double x, CORBA::Double y, CORBA::Double th, OpenHRP::AutoBalancerService::FootstepsSequence_out o_footstep);
  CORBA::Boolean releaseEmergencyStop();
  //
  //
  void autobalancer(AutoBalancer *i_autobalancer);
private:
  AutoBalancer *m_autobalancer;
};				 

#endif
