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
  void servo(ServoController *i_servo);
private:
  ServoController *m_servo;
};

#endif
