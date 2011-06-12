// -*-C++-*-
#ifndef __DATA_LOGGER_SERVICE_IMPL_H__
#define __DATA_LOGGER_SERVICE_IMPL_H__

#include "DataLoggerService.hh"

using namespace OpenHRP;

class DataLogger;
class DataLoggerService_impl
  : public virtual POA_OpenHRP::DataLoggerService,
    public virtual PortableServer::RefCountServantBase
{
public:
  DataLoggerService_impl();
  virtual ~DataLoggerService_impl();

  void setLogger(DataLogger *i_logger) { m_logger = i_logger; }

  CORBA::Boolean add(const char *type, const char *name);
  CORBA::Boolean save(const char *basename);
  CORBA::Boolean clear();
  void maxLength(CORBA::ULong len);
private:
  DataLogger *m_logger;
};

#endif
