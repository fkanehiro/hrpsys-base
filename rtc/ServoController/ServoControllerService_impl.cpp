// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 4; -*-
#include <iostream>
#include "ServoControllerService_impl.h"
#include "ServoController.h"
#include <hrpModel/Body.h>
#include <hrpModel/Link.h>

ServoControllerService_impl::ServoControllerService_impl() : m_servo(NULL)
{
}

ServoControllerService_impl::~ServoControllerService_impl()
{
}

CORBA::Boolean ServoControllerService_impl::setJointAngle(const CORBA::Short id, CORBA::Double jv, CORBA::Double tm)
{
	return m_servo->setJointAngle(id, jv, tm);
}

CORBA::Boolean ServoControllerService_impl::setJointAngles(const OpenHRP::ServoControllerService::dSequence& jvs, CORBA::Double tm)
{
	return m_servo->setJointAngles(jvs, tm);
}

CORBA::Boolean ServoControllerService_impl::getJointAngle(const CORBA::Short id, CORBA::Double &jv)
{
	return m_servo->getJointAngle(id, jv);
}

CORBA::Boolean ServoControllerService_impl::getJointAngles(OpenHRP::ServoControllerService::dSequence_out jvs)
{
	return m_servo->getJointAngles(jvs);
}

CORBA::Boolean ServoControllerService_impl::addJointGroup(const char* gname, const OpenHRP::ServoControllerService::iSequence& ids)
{
    return m_servo->addJointGroup(gname, ids);
}

CORBA::Boolean ServoControllerService_impl::removeJointGroup(const char* gname)
{
    return m_servo->removeJointGroup(gname);
}

CORBA::Boolean ServoControllerService_impl::setJointAnglesOfGroup(const char *gname, const OpenHRP::ServoControllerService::dSequence& jvs, CORBA::Double tm)
{
    return m_servo->setJointAnglesOfGroup(gname, jvs, tm);
}

void ServoControllerService_impl::servo(ServoController *i_servo)
{
  m_servo = i_servo;
} 
