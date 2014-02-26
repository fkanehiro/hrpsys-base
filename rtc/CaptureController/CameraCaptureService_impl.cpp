// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 4; -*-
#include <iostream>
#include "CameraCaptureService_impl.h"
#include "CaptureController.h"

CameraCaptureService_impl::CameraCaptureService_impl(CaptureController *cc) :
	m_cc(cc)
{
}

CameraCaptureService_impl::~CameraCaptureService_impl()
{
}

void CameraCaptureService_impl::take_one_frame()
{
	m_cc->take_one_frame();
}

void CameraCaptureService_impl::start_continuous()
{
	m_cc->start_continuous();
}

void CameraCaptureService_impl::stop_continuous()
{
	m_cc->stop_continuous();
}



