// -*-C++-*-
#ifndef IMPEDANCESERVICESVC_IMPL_H
#define IMPEDANCESERVICESVC_IMPL_H

#include "hrpsys/idl/ImpedanceControllerService.hh"

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

  //
  void impedance(ImpedanceController *i_impedance);
private:
  ImpedanceController *m_impedance;
};				 

#endif
