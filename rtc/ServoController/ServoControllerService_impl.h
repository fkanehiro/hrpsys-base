// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 4; -*-
#ifndef __SERVOCONTROLLER_SERVICE_H__
#define __SERVOCONTROLLER_SERVICE_H__

#include "ServoControllerService.hh"

using namespace OpenHRP;

class ServoController;

class ServoControllerService_impl
	: public virtual POA_OpenHRP::ServoControllerService,
	  public virtual PortableServer::RefCountServantBase
{
public:
	/**
	   \brief constructor
	*/
	ServoControllerService_impl();

	/**
	   \brief destructor
	*/
	virtual ~ServoControllerService_impl();

	CORBA::Boolean setJointAngle(const CORBA::Short id, CORBA::Double jv, CORBA::Double tm);
	CORBA::Boolean setJointAngles(const OpenHRP::ServoControllerService::dSequence& jvs, CORBA::Double tm);
	CORBA::Boolean getJointAngle(const CORBA::Short id, CORBA::Double &jv);
	CORBA::Boolean getJointAngles(OpenHRP::ServoControllerService::dSequence_out jvs);

	CORBA::Boolean addJointGroup(const char* gname, const OpenHRP::ServoControllerService::iSequence& ids);
	CORBA::Boolean removeJointGroup(const char* gname);
	CORBA::Boolean setJointAnglesOfGroup(const char *gname, const OpenHRP::ServoControllerService::dSequence& jvs, CORBA::Double tm);

	CORBA::Boolean setMaxTorque(const CORBA::Short id, const CORBA::Short percentage);
	CORBA::Boolean setReset(const CORBA::Short id);
	CORBA::Boolean getDuration(const CORBA::Short id, CORBA::Double &duration);
	CORBA::Boolean getSpeed(const CORBA::Short id, CORBA::Double &speed);
	CORBA::Boolean getMaxTorque(const CORBA::Short id, CORBA::Short &percentage);
	CORBA::Boolean getTorque(const CORBA::Short id, CORBA::Double &torque);
	CORBA::Boolean getTemperature(const CORBA::Short id, CORBA::Double &temperature);
	CORBA::Boolean getVoltage(const CORBA::Short id, CORBA::Double &voltage);
	CORBA::Boolean servoOn();
	CORBA::Boolean servoOff();

	void servo(ServoController *i_servo);
private:
  ServoController *m_servo;
};

#endif
