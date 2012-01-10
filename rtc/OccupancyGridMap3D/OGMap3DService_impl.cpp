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
