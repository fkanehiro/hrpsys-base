// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 4; -*-
#ifndef THERMOLIMITERSERVICESVC_IMPL_H
#define THERMOLIMITERSERVICESVC_IMPL_H

#include "ThermoLimiterService.hh"

using namespace OpenHRP;

class ThermoLimiter;

class ThermoLimiterService_impl
	: public virtual POA_OpenHRP::ThermoLimiterService,
	  public virtual PortableServer::RefCountServantBase
{
public:
	/**
	   \brief constructor
	*/
	ThermoLimiterService_impl();

	/**
	   \brief destructor
	*/
	virtual ~ThermoLimiterService_impl();
	CORBA::Boolean setParameter(const OpenHRP::ThermoLimiterService::tlParam& i_param);
	CORBA::Boolean getParameter(OpenHRP::ThermoLimiterService::tlParam_out i_param);
	void thermolimiter(ThermoLimiter *i_thermolimiter);
private:
	ThermoLimiter *m_thermolimiter;
};

#endif
