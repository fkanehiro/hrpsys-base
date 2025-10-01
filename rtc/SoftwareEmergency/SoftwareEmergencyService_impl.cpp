// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 4; -*-
#include "SoftwareEmergencyService_impl.h"
#include "SoftwareEmergency.h"

SoftwareEmergencyService_impl::SoftwareEmergencyService_impl(SoftwareEmergency *i_comp) : 
	m_comp(i_comp)
{
}

SoftwareEmergencyService_impl::~SoftwareEmergencyService_impl()
{
}

CORBA::Boolean SoftwareEmergencyService_impl::switchModeTo(::OpenHRP::SoftwareEmergencyService::mode m)
{
  return m_comp->switchModeTo(m);
}

::OpenHRP::SoftwareEmergencyService::mode SoftwareEmergencyService_impl::getMode()
{
  return m_comp->getMode();
}

CORBA::Boolean SoftwareEmergencyService_impl::setButtonStatus(CORBA::Long id, CORBA::Boolean on)
{
  return m_comp->setButtonStatus(id, on);
}
