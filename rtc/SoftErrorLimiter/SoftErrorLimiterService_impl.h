// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 4; -*-
#ifndef __SOFT_ERROR_LIMITER_SERVICE_H__
#define __SOFT_ERROR_LIMITER_SERVICE_H__

#include "SoftErrorLimiterService.hh"
#include "robot.h"

class SoftErrorLimiterService_impl
	: public virtual POA_OpenHRP::SoftErrorLimiterService,
	  public virtual PortableServer::RefCountServantBase
{
public:
	/**
	   \brief constructor
	*/
	SoftErrorLimiterService_impl();

	/**
	   \brief destructor
	*/
	virtual ~SoftErrorLimiterService_impl();

    void setServoErrorLimit(const char *jname, double limit);

    //
    void setRobot(boost::shared_ptr<robot>& i_robot) { m_robot = i_robot; }

private:
    boost::shared_ptr<robot> m_robot;
};

#endif
