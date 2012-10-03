// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 4; -*-
#include <iostream>
#include "GraspControllerService_impl.h"
#include "GraspController.h"

GraspControllerService_impl::GraspControllerService_impl()
{
}

GraspControllerService_impl::~GraspControllerService_impl()
{
}

bool GraspControllerService_impl::startGrasp(const char *name, double target_error)
{
	return m_grasp->startGrasp(name, target_error);
}

bool GraspControllerService_impl::stopGrasp(const char *name)
{
	return m_grasp->stopGrasp(name);
}

void GraspControllerService_impl::grasp(GraspController *i_grasp)
{
	m_grasp = i_grasp;
} 
