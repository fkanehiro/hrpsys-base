// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 4; -*-
#ifndef __EMERGENCYSTOPPER_SERVICE_H__
#define __EMERGENCYSTOPPER_SERVICE_H__

#include "EmergencyStopperService.hh"

class EmergencyStopperService_impl
	: public virtual POA_OpenHRP::EmergencyStopperService,
	  public virtual PortableServer::RefCountServantBase
{
public:
	/**
	   \brief constructor
	*/
	EmergencyStopperService_impl();

	/**
	   \brief destructor
	*/
	virtual ~EmergencyStopperService_impl();

	void echo(const char *msg);
private:
};

#endif
