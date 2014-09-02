// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 4; -*-
#include <iostream>
#include "KalmanFilterService_impl.h"
#include "KalmanFilter.h"

KalmanFilterService_impl::KalmanFilterService_impl()
{
}

KalmanFilterService_impl::~KalmanFilterService_impl()
{
}

bool KalmanFilterService_impl::setKalmanFilterParam(const OpenHRP::KalmanFilterService::KalmanFilterParam& i_param)
{
	return m_kalman->setKalmanFilterParam(i_param);
}

bool KalmanFilterService_impl::getKalmanFilterParam(OpenHRP::KalmanFilterService::KalmanFilterParam& i_param)
{
	i_param = OpenHRP::KalmanFilterService::KalmanFilterParam();
	return m_kalman->getKalmanFilterParam(i_param);
}

void KalmanFilterService_impl::kalman(KalmanFilter *i_kalman)
{
	m_kalman = i_kalman;
}


