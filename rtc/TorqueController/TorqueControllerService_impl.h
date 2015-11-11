// -*- coding:utf-8-unix; mode: c++; indent-tabs-mode: nil; c-basic-offset: 2; -*-
#ifndef __NULL_SERVICE_H__
#define __NULL_SERVICE_H__

#include "TorqueControllerService.hh"

using namespace OpenHRP;

class TorqueController;

class TorqueControllerService_impl
	: public virtual POA_OpenHRP::TorqueControllerService,
	  public virtual PortableServer::RefCountServantBase
{
public:
	TorqueControllerService_impl();
	virtual ~TorqueControllerService_impl();

	CORBA::Boolean enableTorqueController(const char *jointName);
	CORBA::Boolean enableMultipleTorqueControllers(const OpenHRP::TorqueControllerService::StrSequence& jnames);
	CORBA::Boolean disableTorqueController(const char *jointName);
	CORBA::Boolean disableMultipleTorqueControllers(const OpenHRP::TorqueControllerService::StrSequence& jnames);
	
	CORBA::Boolean startTorqueControl(const char *jointName);
	CORBA::Boolean startMultipleTorqueControls(const OpenHRP::TorqueControllerService::StrSequence& jnames);
	CORBA::Boolean stopTorqueControl(const char *jointName);
	CORBA::Boolean stopMultipleTorqueControls(const OpenHRP::TorqueControllerService::StrSequence& jnames);
	CORBA::Boolean setReferenceTorque(const char *jointName, double tauRef);
	CORBA::Boolean setMultipleReferenceTorques(const OpenHRP::TorqueControllerService::StrSequence& jnames, const OpenHRP::TorqueControllerService::dSequence& tauRefs);
	CORBA::Boolean setTorqueControllerParam(const OpenHRP::TorqueControllerService::torqueControllerParam& t_param);
	
	void torque_controller(TorqueController *i_torque_controller);
private:
	TorqueController *m_torque_controller;
};

#endif
