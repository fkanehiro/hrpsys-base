#include "PCDLoaderService_impl.h"
#include "PCDLoader.h"

PCDLoaderService_impl::PCDLoaderService_impl() : m_comp(NULL)
{
}

PCDLoaderService_impl::~PCDLoaderService_impl()
{
}


void PCDLoaderService_impl::setComp(PCDLoader *i_comp)
{
    m_comp = i_comp;
}

::CORBA::Boolean PCDLoaderService_impl::load(const char* filename, const char* label)
{
    return m_comp->load(filename, label);
}

void PCDLoaderService_impl::offset(const char* label, CORBA::Double cx, CORBA::Double cy, CORBA::Double cz,
                                   CORBA::Double ox, CORBA::Double oy, CORBA::Double oz,
                                   CORBA::Double r, CORBA::Double p, CORBA::Double y)
{
    return m_comp->offset(label, hrp::Vector3(cx, cy, cz), hrp::Vector3(ox, oy, oz), hrp::rotFromRpy(r, p, y));
}
