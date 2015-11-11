// -*- coding:utf-8-unix; mode: c++; indent-tabs-mode: nil; c-basic-offset: 2; -*-
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
