// -*-C++-*-
#ifndef WholeBodyMasterSlaveSERVICESVC_IMPL_H
#define WholeBodyMasterSlaveSERVICESVC_IMPL_H

#include "hrpsys/idl/WholeBodyMasterSlaveService.hh"

using namespace OpenHRP;

class WholeBodyMasterSlave;

class WholeBodyMasterSlaveService_impl
  : public virtual POA_OpenHRP::WholeBodyMasterSlaveService,
    public virtual PortableServer::RefCountServantBase
{
public:
  WholeBodyMasterSlaveService_impl();
  virtual ~WholeBodyMasterSlaveService_impl();
  CORBA::Boolean startCountDownForWholeBodyMasterSlave(const CORBA::Double sec);
  CORBA::Boolean startWholeBodyMasterSlave();
  CORBA::Boolean stopWholeBodyMasterSlave();
  CORBA::Boolean pauseWholeBodyMasterSlave();
  CORBA::Boolean resumeWholeBodyMasterSlave();
  void setWholeBodyMasterSlaveParam(const OpenHRP::WholeBodyMasterSlaveService::WholeBodyMasterSlaveParam& i_param);
  void getWholeBodyMasterSlaveParam(OpenHRP::WholeBodyMasterSlaveService::WholeBodyMasterSlaveParam_out i_param);
  //
  void wholebodymasterslave(WholeBodyMasterSlave *i_wholebodymasterslave);
private:
  WholeBodyMasterSlave *m_wholebodymasterslave;
};				 

#endif
