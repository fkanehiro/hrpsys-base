// -*-C++-*-
#ifndef ABSOLUTEFORCESENSORSERVICESVC_IMPL_H
#define ABSOLUTEFORCESENSORSERVICESVC_IMPL_H

#include "AbsoluteForceSensorService.hh"

using namespace OpenHRP;

class AbsoluteForceSensor;

class AbsoluteForceSensorService_impl 
  : public virtual POA_OpenHRP::AbsoluteForceSensorService,
    public virtual PortableServer::RefCountServantBase
{
public:
  AbsoluteForceSensorService_impl();
  virtual ~AbsoluteForceSensorService_impl();
  //
  CORBA::Boolean setForceMomentOffsetParam(const char* i_name_, const OpenHRP::AbsoluteForceSensorService::forcemomentOffsetParam &i_param_);
  CORBA::Boolean getForceMomentOffsetParam(const char *i_name_, OpenHRP::AbsoluteForceSensorService::forcemomentOffsetParam_out i_param_);
  //
  void absfsensor(AbsoluteForceSensor *i_absfsensor);
private:
  AbsoluteForceSensor *m_absfsensor;
};				 

#endif
