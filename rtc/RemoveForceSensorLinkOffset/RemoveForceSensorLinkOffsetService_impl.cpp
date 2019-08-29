// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 4; -*-
#include <iostream>
#include "RemoveForceSensorLinkOffsetService_impl.h"
#include "RemoveForceSensorLinkOffset.h"

RemoveForceSensorLinkOffsetService_impl::RemoveForceSensorLinkOffsetService_impl() : m_rmfsoff(NULL)
{
}

RemoveForceSensorLinkOffsetService_impl::~RemoveForceSensorLinkOffsetService_impl()
{
}

CORBA::Boolean RemoveForceSensorLinkOffsetService_impl::setForceMomentOffsetParam(const char* i_name_, const OpenHRP::RemoveForceSensorLinkOffsetService::forcemomentOffsetParam &i_param_)
{
  return m_rmfsoff->setForceMomentOffsetParam(std::string(i_name_), i_param_);
}

CORBA::Boolean RemoveForceSensorLinkOffsetService_impl::getForceMomentOffsetParam(const char *i_name_, OpenHRP::RemoveForceSensorLinkOffsetService::forcemomentOffsetParam_out i_param_)
{
  i_param_ = new OpenHRP::RemoveForceSensorLinkOffsetService::forcemomentOffsetParam();
  i_param_->force_offset.length(3);
  i_param_->moment_offset.length(3);
  i_param_->link_offset_centroid.length(3);
  return m_rmfsoff->getForceMomentOffsetParam(std::string(i_name_), *i_param_);
}

CORBA::Boolean RemoveForceSensorLinkOffsetService_impl::loadForceMomentOffsetParams(const char *filename)
{
	return m_rmfsoff->loadForceMomentOffsetParams(std::string(filename));
};

CORBA::Boolean RemoveForceSensorLinkOffsetService_impl::dumpForceMomentOffsetParams(const char *filename)
{
	return m_rmfsoff->dumpForceMomentOffsetParams(std::string(filename));
};

CORBA::Boolean RemoveForceSensorLinkOffsetService_impl::removeForceSensorOffset(const ::OpenHRP::RemoveForceSensorLinkOffsetService::StrSequence& names, CORBA::Double tm)
{
	return m_rmfsoff->removeForceSensorOffset(names, tm);
}

void RemoveForceSensorLinkOffsetService_impl::rmfsoff(RemoveForceSensorLinkOffset *i_rmfsoff)
{
	m_rmfsoff = i_rmfsoff;
}

