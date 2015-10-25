// -*- coding:utf-8-unix; mode: c++; indent-tabs-mode: nil; c-basic-offset: 2; -*-
#ifndef __GRAP_CONTROLLER_SERVICE_H__
#define __GRAP_CONTROLLER_SERVICE_H__

#include "GraspControllerService.hh"

class GraspController;

class GraspControllerService_impl
	: public virtual POA_OpenHRP::GraspControllerService,
	  public virtual PortableServer::RefCountServantBase
{
public:
	/**
	   \brief constructor
	*/
	GraspControllerService_impl();

	/**
	   \brief destructor
	*/
	virtual ~GraspControllerService_impl();

    bool startGrasp(const char *name, double target_error);
    bool stopGrasp(const char *name);
    //
	void grasp(GraspController *i_grasp);
private:
  GraspController *m_grasp;
};

#endif
