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

bool KalmanFilterService_impl::SetKalmanFilterParam(double Q_angle, double Q_rate, double R_angle)
{
	//std::cout << "KalmanFilterService: " << std::endl;
	return m_kalman->SetKalmanFilterParam(Q_angle, Q_rate, R_angle);
}

void KalmanFilterService_impl::kalman(KalmanFilter *i_kalman)
{
	m_kalman = i_kalman;
}


