// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 4; -*-
#include <iostream>
#include "CameraCaptureService_impl.h"
#include "VideoCapture.h"

CameraCaptureService_impl::CameraCaptureService_impl(VideoCapture *vc) :
	m_vc(vc)
{
}

CameraCaptureService_impl::~CameraCaptureService_impl()
{
}

void CameraCaptureService_impl::take_one_frame()
{
	m_vc->take_one_frame();
}

void CameraCaptureService_impl::start_continuous()
{
	m_vc->start_continuous();
}

void CameraCaptureService_impl::stop_continuous()
{
	m_vc->stop_continuous();
}



