// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 4; -*-
#ifndef __KALMANFILTER_SERVICE_H__
#define __KALMANFILTER_SERVICE_H__

#include "StabilizerService.hh"

class StabilizerService_impl
	: public virtual POA_OpenHRP::StabilizerService,
	  public virtual PortableServer::RefCountServantBase
{
public:
	/**
	   \brief constructor
	*/
	StabilizerService_impl();

	/**
	   \brief destructor
	*/
	virtual ~StabilizerService_impl();

	bool dummy();
private:
};

#endif
