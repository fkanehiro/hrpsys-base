// -*- coding:utf-8-unix; mode: c++; indent-tabs-mode: nil; c-basic-offset: 2; -*-
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
