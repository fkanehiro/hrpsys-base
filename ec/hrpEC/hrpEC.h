// -*- C++ -*-
#ifndef hrpEC_h
#define hrpEC_h

#include <rtm/RTC.h>

#include <coil/Mutex.h>
#include <coil/Condition.h>
#include <coil/Task.h>

#include <rtm/Manager.h>
#include <rtm/PeriodicExecutionContext.h>

#include "ExecutionProfileService.hh"

namespace RTC
{
  class hrpExecutionContext
      : public virtual PeriodicExecutionContext,
        public virtual POA_OpenHRP::ExecutionProfileService,
        public virtual PortableServer::RefCountServantBase
  {
  public:
    hrpExecutionContext();
    virtual ~hrpExecutionContext();
    virtual int svc(void);

    OpenHRP::ExecutionProfileService::Profile *getProfile();
    void resetProfile();
  private:
    OpenHRP::ExecutionProfileService::Profile m_profile;
    struct timeval m_tv;
  };
};

extern "C"
{
  void hrpExecutionContextInit(RTC::Manager* manager);
};

#endif // hrpEC_h

