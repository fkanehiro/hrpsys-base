// -*-C++-*-
#ifndef IMPEDANCESERVICESVC_IMPL_H
#define IMPEDANCESERVICESVC_IMPL_H

#include "ImpedanceControllerService.hh"

using namespace OpenHRP;

class ImpedanceController;

class ImpedanceControllerService_impl 
  : public virtual POA_OpenHRP::ImpedanceControllerService,
    public virtual PortableServer::RefCountServantBase
{
public:
  ImpedanceControllerService_impl();
  virtual ~ImpedanceControllerService_impl();
  //
  CORBA::Boolean startImpedanceController(const char *i_name_);
  CORBA::Boolean startImpedanceControllerNoWait(const char *i_name_);
  CORBA::Boolean stopImpedanceController(const char *i_name_);
  CORBA::Boolean stopImpedanceControllerNoWait(const char *i_name_);
  CORBA::Boolean setImpedanceControllerParam(const char *i_name_, const OpenHRP::ImpedanceControllerService::impedanceParam &i_param_);
  CORBA::Boolean getImpedanceControllerParam(const char *i_name_, OpenHRP::ImpedanceControllerService::impedanceParam_out i_param_);
  void waitImpedanceControllerTransition(const char *i_name_);
  void startObjectTurnaroundDetection(const CORBA::Double i_ref_diff_wrench, const CORBA::Double i_max_time, const OpenHRP::ImpedanceControllerService::StrSequence& i_ee_names);
  OpenHRP::ImpedanceControllerService::DetectorMode checkObjectTurnaroundDetection();
  CORBA::Boolean setObjectTurnaroundDetectorParam(const OpenHRP::ImpedanceControllerService::objectTurnaroundDetectorParam &i_param_);
  CORBA::Boolean getObjectTurnaroundDetectorParam(OpenHRP::ImpedanceControllerService::objectTurnaroundDetectorParam& i_param_);
  CORBA::Boolean getObjectForcesMoments(OpenHRP::ImpedanceControllerService::Dbl3Sequence_out o_forces, OpenHRP::ImpedanceControllerService::Dbl3Sequence_out o_moments, OpenHRP::ImpedanceControllerService::DblSequence3_out o_3dofwrench);

  //
  void impedance(ImpedanceController *i_impedance);
private:
  ImpedanceController *m_impedance;
};				 

#endif
