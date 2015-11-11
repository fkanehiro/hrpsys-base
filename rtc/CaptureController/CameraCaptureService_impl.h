// -*- coding:utf-8-unix; mode: c++; indent-tabs-mode: nil; c-basic-offset: 2; -*-
#ifndef __CAMERA_CAPTURE_SERVICE_H__
#define __CAMERA_CAPTURE_SERVICE_H__

#include "Img.hh"

class CaptureController;

class CameraCaptureService_impl
	: public virtual POA_Img::CameraCaptureService,
	  public virtual PortableServer::RefCountServantBase
{
public:
	/**
	   \brief constructor
	*/
	CameraCaptureService_impl(CaptureController *cc);

	/**
	   \brief destructor
	*/
	virtual ~CameraCaptureService_impl();

    void take_one_frame();
    void start_continuous();
    void stop_continuous();
private:
	CaptureController *m_cc;
};

#endif
