// -*- coding:utf-8-unix; mode: c++; indent-tabs-mode: nil; c-basic-offset: 2; -*-
#include <iostream>
#include "StabilizerService_impl.h"
#include "Stabilizer.h"

StabilizerService_impl::StabilizerService_impl() : m_stabilizer(NULL)
{
}

StabilizerService_impl::~StabilizerService_impl()
{
}

void StabilizerService_impl::startStabilizer(void)
{
	m_stabilizer->startStabilizer();
}

void StabilizerService_impl::stopStabilizer(void)
{
	m_stabilizer->stopStabilizer();
}

void StabilizerService_impl::getParameter(OpenHRP::StabilizerService::stParam_out i_param)
{
  i_param = new OpenHRP::StabilizerService::stParam();
  return m_stabilizer->getParameter(*i_param);
};

void StabilizerService_impl::setParameter(const OpenHRP::StabilizerService::stParam& i_stp)
{
	m_stabilizer->setParameter(i_stp);
}

bool StabilizerService_impl::dummy()
{
	std::cout << "StabilizerService: " << std::endl;
}

void StabilizerService_impl::stabilizer(Stabilizer *i_stabilizer)
{
  m_stabilizer = i_stabilizer;
} 
