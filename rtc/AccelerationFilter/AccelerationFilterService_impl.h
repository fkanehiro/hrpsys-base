// -*-C++-*-
/*!
 * @file  ../AccelerationFilterService_impl.h
 * @brief Service implementation header of ../AccelerationFilterService.idl
 *
 */

#include "hrpsys/idl/AccelerationFilterService.hh"


#ifndef ACCELERATIONFILTERSERVICE_IMPL_H
#define ACCELERATIONFILTERSERVICE_IMPL_H

class AccelerationFilter;

 
/*
 * Example class implementing IDL interface OpenHRP::AccelerationFilterService
 */
class AccelerationFilterService_impl
    : public virtual POA_OpenHRP::AccelerationFilterService,
      public virtual PortableServer::RefCountServantBase
{
private:
    // Make sure all instances are built on the heap by making the
    // destructor non-public
    //virtual ~OpenHRP_AccelerationFilterService_impl();
    AccelerationFilter *m_instance;

public:
    // standard constructor
    AccelerationFilterService_impl();
    virtual ~AccelerationFilterService_impl();

    // attributes and operations
    ::CORBA::Boolean setAccelerationFilterParam(const ::OpenHRP::AccelerationFilterService::AccelerationFilterParam& i_param);
    ::CORBA::Boolean getAccelerationFilterParam(::OpenHRP::AccelerationFilterService::AccelerationFilterParam_out  i_param);
    ::CORBA::Boolean resetFilter(OpenHRP::AccelerationFilterService::ControlMode mode,
                                 const ::OpenHRP::AccelerationFilterService::DblArray3 vel);
    //
    void setInstance(AccelerationFilter *i_instance);
};



#endif // ACCELERATIONFILTERSERVICE_IMPL_H


