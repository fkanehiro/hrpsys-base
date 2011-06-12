#include "TimeKeeperService_impl.h"
#include "StateHolder.h"

TimeKeeperService_impl::TimeKeeperService_impl() : m_comp(NULL)
{
}

TimeKeeperService_impl::~TimeKeeperService_impl()
{
}

void TimeKeeperService_impl::sleep(CORBA::Double tm)
{
    m_comp->wait(tm);
}
