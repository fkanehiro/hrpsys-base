// -*- coding:utf-8-unix; mode: c++; indent-tabs-mode: nil; c-basic-offset: 2; -*-
#include "DataLoggerService_impl.h"
#include "DataLogger.h"

DataLoggerService_impl::DataLoggerService_impl() : m_logger(NULL)
{
}

DataLoggerService_impl::~DataLoggerService_impl()
{
}

CORBA::Boolean DataLoggerService_impl::add(const char *type, const char *name)
{
  return m_logger->add(type, name);
}

CORBA::Boolean DataLoggerService_impl::save(const char *basename)
{
  return m_logger->save(basename);
}

CORBA::Boolean DataLoggerService_impl::clear()
{
  return m_logger->clear();
}

void DataLoggerService_impl::maxLength(CORBA::ULong len)
{
  m_logger->maxLength(len);
}


