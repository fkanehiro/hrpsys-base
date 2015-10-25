// -*- coding:utf-8-unix; mode: c++; indent-tabs-mode: nil; c-basic-offset: 2; -*-
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



