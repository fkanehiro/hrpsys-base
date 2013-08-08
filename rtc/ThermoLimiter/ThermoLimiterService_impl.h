// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 4; -*-
#ifndef __NULL_SERVICE_H__
#define __NULL_SERVICE_H__

#include "NullService.hh"

class NullService_impl
	: public virtual POA_OpenHRP::NullService,
	  public virtual PortableServer::RefCountServantBase
{
public:
	/**
	   \brief constructor
	*/
	NullService_impl();

	/**
	   \brief destructor
	*/
	virtual ~NullService_impl();

	void echo(const char *msg);
private:
};

#endif
