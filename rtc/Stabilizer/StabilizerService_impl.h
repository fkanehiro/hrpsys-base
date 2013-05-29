// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 4; -*-
#ifndef __KALMANFILTER_SERVICE_H__
#define __KALMANFILTER_SERVICE_H__

#include "StabilizerService.hh"

class Stabilizer;

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

	void startStabilizer(void);
	void stopStabilizer(void);
	void stabilizer(Stabilizer *i_stabilizer);

	bool dummy();
private:
	Stabilizer *m_stabilizer;
};

#endif
