// -*- coding:utf-8-unix; mode: c++; indent-tabs-mode: nil; c-basic-offset: 2; -*-
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



