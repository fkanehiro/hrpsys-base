// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 4; -*-
#ifndef __SOFTWARE_SERVICE_SERVICE_H__
#define __SOFTWARE_SERVICE_SERVICE_H__

#include "hrpsys/idl/SoftwareEmergencyService.hh"
class SoftwareEmergency;

class SoftwareEmergencyService_impl
	: public virtual POA_OpenHRP::SoftwareEmergencyService,
	  public virtual PortableServer::RefCountServantBase
{
public:
	/**
	   \brief constructor
	*/
	SoftwareEmergencyService_impl(SoftwareEmergency *i_swinger);

	/**
	   \brief destructor
	*/
	virtual ~SoftwareEmergencyService_impl();

	// 
	CORBA::Boolean switchModeTo(::OpenHRP::SoftwareEmergencyService::mode m);
	::OpenHRP::SoftwareEmergencyService::mode getMode();
	
	CORBA::Boolean setButtonStatus(CORBA::Long id, CORBA::Boolean on);
private:
	SoftwareEmergency *m_comp;
};

#endif
