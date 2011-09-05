// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 4; -*-
#ifndef __OGMap3DService_H__
#define __OGMap3DService_H__

#include "OGMap3DService.hh"

class OccupancyGridMap3D;

class OGMap3DService_impl
  : public virtual POA_OpenHRP::OGMap3DService,
    public virtual PortableServer::RefCountServantBase
{
public:
  /**
     \brief constructor
  */
  OGMap3DService_impl(OccupancyGridMap3D *i_comp);

  /**
     \brief destructor
  */
  virtual ~OGMap3DService_impl();

  OpenHRP::OGMap3D* getOGMap3D(const OpenHRP::AABB& region);

private:
  OccupancyGridMap3D *m_comp;
};

#endif
