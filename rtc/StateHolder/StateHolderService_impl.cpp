// -*- coding:utf-8-unix; mode: c++; indent-tabs-mode: nil; c-basic-offset: 2; -*-
#include "StateHolderService_impl.h"
#include "StateHolder.h"

StateHolderService_impl::StateHolderService_impl() : m_comp(NULL)
{
}

StateHolderService_impl::~StateHolderService_impl()
{
}

void StateHolderService_impl::goActual()
{
    if (!m_comp) return;

    m_comp->goActual();
}

void StateHolderService_impl::getCommand(OpenHRP::StateHolderService::Command_out com)
{
    com = new OpenHRP::StateHolderService::Command;
    m_comp->getCommand(*com);
}
