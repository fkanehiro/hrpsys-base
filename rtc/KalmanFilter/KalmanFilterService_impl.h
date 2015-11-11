// -*- coding:utf-8-unix; mode: c++; indent-tabs-mode: nil; c-basic-offset: 2; -*-
#ifndef __KALMANFILTER_SERVICE_H__
#define __KALMANFILTER_SERVICE_H__

#include "KalmanFilterService.hh"

class KalmanFilter;

class KalmanFilterService_impl
	: public virtual POA_OpenHRP::KalmanFilterService,
	  public virtual PortableServer::RefCountServantBase
{
public:
	/**
	   \brief constructor
	*/
	KalmanFilterService_impl();

	/**
	   \brief destructor
	*/
	virtual ~KalmanFilterService_impl();

	bool setKalmanFilterParam(const OpenHRP::KalmanFilterService::KalmanFilterParam& i_param);
	bool getKalmanFilterParam(OpenHRP::KalmanFilterService::KalmanFilterParam& i_param);
	bool resetKalmanFilterState();

	void kalman(KalmanFilter *i_kalman);

private:
	KalmanFilter *m_kalman;
};

#endif
