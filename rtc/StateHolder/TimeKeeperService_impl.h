// -*-C++-*-
#ifndef TIMEKEEPERSERVICE_IMPL_H
#define TIMEKEEPERSERVICE_IMPL_H

#include "TimeKeeperService.hh"

using namespace OpenHRP;

class StateHolder;

class TimeKeeperService_impl 
  : public virtual POA_OpenHRP::TimeKeeperService,
    public virtual PortableServer::RefCountServantBase
{
public:
  TimeKeeperService_impl();
  virtual ~TimeKeeperService_impl();
  void setComponent(StateHolder *i_comp) { m_comp = i_comp; }
  void sleep(CORBA::Double tm);
private:
  StateHolder *m_comp;
};				 

#endif
