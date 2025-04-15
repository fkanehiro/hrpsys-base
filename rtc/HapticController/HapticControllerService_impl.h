// -*-C++-*-
#ifndef HAPTICCONTROLLERSERVICESVC_IMPL_H
#define HAPTICCONTROLLERSERVICESVC_IMPL_H

#include "hrpsys/idl/HRPDataTypes.hh"
#include "hrpsys/idl/HapticControllerService.hh"

using namespace OpenHRP;

class HapticController;

class HapticControllerService_impl
  : public virtual POA_OpenHRP::HapticControllerService,
    public virtual PortableServer::RefCountServantBase
{
public:
  HapticControllerService_impl();
  virtual ~HapticControllerService_impl();
  CORBA::Boolean startHapticController();
  CORBA::Boolean stopHapticController();
  CORBA::Boolean pauseHapticController();
  CORBA::Boolean resumeHapticController();
  void resetOdom();
  void setParams(const OpenHRP::HapticControllerService::HapticControllerParam& i_param);
  void getParams(OpenHRP::HapticControllerService::HapticControllerParam_out i_param);
  //
  void hapticcontroller(HapticController *i_hapticcontroller);
private:
  HapticController *m_hapticcontroller;
};				 

#endif
