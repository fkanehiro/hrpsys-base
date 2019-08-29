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
