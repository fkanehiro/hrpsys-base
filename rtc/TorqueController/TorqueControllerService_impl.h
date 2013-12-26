// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 4; -*-
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

	CORBA::Boolean startTorqueControl(const char *jointName);
	CORBA::Boolean startMultipleTorqueControls(const OpenHRP::TorqueControllerService::StrSequence& jnames);
	CORBA::Boolean stopTorqueControl(const char *jointName);
	CORBA::Boolean stopMultipleTorqueControls(const OpenHRP::TorqueControllerService::StrSequence& jnames);
	CORBA::Boolean setReferenceTorque(const char *jointName, double tauRef);
	CORBA::Boolean setMultipleReferenceTorques(const OpenHRP::TorqueControllerService::StrSequence& jnames, const OpenHRP::TorqueControllerService::dSequence& tauRefs);
	
	void torque_controller(TorqueController *i_torque_controller);
private:
	TorqueController *m_torque_controller;
};

#endif
