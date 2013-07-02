// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 4; -*-
#include <iostream>
#include "AbsoluteForceSensorService_impl.h"
#include "AbsoluteForceSensor.h"

AbsoluteForceSensorService_impl::AbsoluteForceSensorService_impl() : m_absfsensor(NULL)
{
}

AbsoluteForceSensorService_impl::~AbsoluteForceSensorService_impl()
{
}

CORBA::Boolean AbsoluteForceSensorService_impl::setForceMomentOffsetParam(const char* i_name_, const OpenHRP::AbsoluteForceSensorService::forcemomentOffsetParam &i_param_)
{
  return m_absfsensor->setForceMomentOffsetParam(std::string(i_name_), i_param_);
}

CORBA::Boolean AbsoluteForceSensorService_impl::getForceMomentOffsetParam(const char *i_name_, OpenHRP::AbsoluteForceSensorService::forcemomentOffsetParam_out i_param_)
{
  i_param_ = new OpenHRP::AbsoluteForceSensorService::forcemomentOffsetParam();
  i_param_->force_offset.length(3);
  i_param_->moment_offset.length(3);
  i_param_->link_offset_centroid.length(3);
  return m_absfsensor->getForceMomentOffsetParam(std::string(i_name_), *i_param_);
}

void AbsoluteForceSensorService_impl::absfsensor(AbsoluteForceSensor *i_absfsensor)
{
	m_absfsensor = i_absfsensor;
}

