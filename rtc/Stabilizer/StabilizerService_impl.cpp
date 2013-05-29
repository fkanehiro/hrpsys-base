// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 4; -*-
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

bool StabilizerService_impl::dummy()
{
	std::cout << "StabilizerService: " << std::endl;
}

void StabilizerService_impl::stabilizer(Stabilizer *i_stabilizer)
{
  m_stabilizer = i_stabilizer;
} 
