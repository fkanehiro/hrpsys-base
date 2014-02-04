// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 4; -*-
#ifndef __CAMERA_CAPTURE_SERVICE_H__
#define __CAMERA_CAPTURE_SERVICE_H__

#include "Img.hh"

class VideoCapture;

class CameraCaptureService_impl
	: public virtual POA_Img::CameraCaptureService,
	  public virtual PortableServer::RefCountServantBase
{
public:
	/**
	   \brief constructor
	*/
	CameraCaptureService_impl(VideoCapture *vc);

	/**
	   \brief destructor
	*/
	virtual ~CameraCaptureService_impl();

    void take_one_frame();
    void start_continuous();
    void stop_continuous();
private:
	VideoCapture *m_vc;
};

#endif
