// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 4; -*-
#ifndef __NULL_SERVICE_H__
#define __NULL_SERVICE_H__

#include "SampleComponentService.hh"

class SampleComponent;

class SampleComponent_impl
	: public virtual POA_OpenHRP::SampleComponentService,
	  public virtual PortableServer::RefCountServantBase
{
public:
	/**
	   \brief constructor
	*/
	SampleComponent_impl();

	/**
	   \brief destructor
	*/
	virtual ~SampleComponent_impl();

	void echo(const char *msg);

	void sample(SampleComponent *i_sample);

private:
	SampleComponent *m_sample;
};

#endif
