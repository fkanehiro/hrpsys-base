// -*-C++-*-
#ifndef REMOVEFORCESENSORLINKOFFSETSERVICE_IMPL_H
#define REMOVEFORCESENSORLINKOFFSETSERVICE_IMPL_H

#include "hrpsys/idl/RemoveForceSensorLinkOffsetService.hh"

using namespace OpenHRP;

class RemoveForceSensorLinkOffset;

class RemoveForceSensorLinkOffsetService_impl 
  : public virtual POA_OpenHRP::RemoveForceSensorLinkOffsetService,
    public virtual PortableServer::RefCountServantBase
{
public:
  RemoveForceSensorLinkOffsetService_impl();
  virtual ~RemoveForceSensorLinkOffsetService_impl();
  //
  CORBA::Boolean setForceMomentOffsetParam(const char* i_name_, const OpenHRP::RemoveForceSensorLinkOffsetService::forcemomentOffsetParam &i_param_);
  CORBA::Boolean getForceMomentOffsetParam(const char *i_name_, OpenHRP::RemoveForceSensorLinkOffsetService::forcemomentOffsetParam_out i_param_);
  CORBA::Boolean loadForceMomentOffsetParams(const char *fiename);
  CORBA::Boolean dumpForceMomentOffsetParams(const char *fiename);
  CORBA::Boolean removeForceSensorOffset(const ::OpenHRP::RemoveForceSensorLinkOffsetService::StrSequence& names, CORBA::Double tm);
  //
  void rmfsoff(RemoveForceSensorLinkOffset *i_rmfsoff);
private:
  RemoveForceSensorLinkOffset *m_rmfsoff;
};				 

#endif
