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
  CORBA::Boolean setImpedanceControllerParam(const OpenHRP::ImpedanceControllerService::impedanceParam &i_param_);
  CORBA::Boolean deleteImpedanceController(const char *i_name_);
  //
  void impedance(ImpedanceController *i_impedance);
private:
  ImpedanceController *m_impedance;
};				 

#endif
