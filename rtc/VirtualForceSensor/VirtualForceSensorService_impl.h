// -*-C++-*-
#ifndef IMPEDANCESERVICESVC_IMPL_H
#define IMPEDANCESERVICESVC_IMPL_H

#include "VirtualForceSensorService.hh"

using namespace OpenHRP;

class VirtualForceSensor;

class VirtualForceSensorService_impl 
  : public virtual POA_OpenHRP::VirtualForceSensorService,
    public virtual PortableServer::RefCountServantBase
{
public:
  VirtualForceSensorService_impl();
  virtual ~VirtualForceSensorService_impl();
  //
  CORBA::Boolean removeVirtualForceSensorOffset(const char *sensorName);
  //
  void vfsensor(VirtualForceSensor *i_vfsensor);
private:
  VirtualForceSensor *m_vfsensor;
};				 

#endif
