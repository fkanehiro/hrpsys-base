// -*- coding:utf-8-unix; mode: c++; indent-tabs-mode: nil; c-basic-offset: 2; -*-
#include "OGMap3DService_impl.h"
#include "OccupancyGridMap3D.h"

using namespace RTC;

OGMap3DService_impl::OGMap3DService_impl(OccupancyGridMap3D *i_comp)
{
    m_comp = i_comp;
}

OGMap3DService_impl::~OGMap3DService_impl()
{
}

OpenHRP::OGMap3D* OGMap3DService_impl::getOGMap3D(const OpenHRP::AABB& region)
{
    return m_comp->getOGMap3D(region);
}

void OGMap3DService_impl::save(const char *filename)
{
    m_comp->save(filename);
}

void OGMap3DService_impl::clear()
{
    m_comp->clear();
}
