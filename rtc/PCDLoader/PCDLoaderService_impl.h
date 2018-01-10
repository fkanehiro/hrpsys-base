#ifndef __PCD_LOADER_SERVICE_IMPL_H__
#define __PCD_LOADER_SERVICE_IMPL_H__

#include "hrpsys/idl/PCDLoaderService.hh"

class PCDLoader;

class PCDLoaderService_impl
    : public virtual POA_OpenHRP::PCDLoaderService,
      public virtual PortableServer::RefCountServantBase
{
 public:
    /**
       \brief constructor
    */
    PCDLoaderService_impl();
    
    /**
       \brief destructor
    */
    virtual ~PCDLoaderService_impl();
    
    void setComp(PCDLoader *i_comp);
    //
    ::CORBA::Boolean load(const char* filename, const char* label);
 private:
    PCDLoader *m_comp;
};
    
#endif
