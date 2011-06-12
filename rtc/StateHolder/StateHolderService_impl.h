// -*-C++-*-
#ifndef STATEHOLDERSERVICE_IMPL_H
#define STATEHOLDERSERVICE_IMPL_H

#include "StateHolderService.hh"

using namespace OpenHRP;

class StateHolder;

class StateHolderService_impl 
  : public virtual POA_OpenHRP::StateHolderService,
    public virtual PortableServer::RefCountServantBase
{
public:
  StateHolderService_impl();
  virtual ~StateHolderService_impl();
  void setComponent(StateHolder *i_comp) { m_comp = i_comp; }
  void goActual();
  void getCommand(OpenHRP::StateHolderService::Command_out com);
private:
  StateHolder *m_comp;
};				 

#endif
