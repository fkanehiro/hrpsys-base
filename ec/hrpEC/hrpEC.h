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
    //
    bool enterRT();
    bool exitRT();
    bool waitForNextPeriod();
  private:
    template <class T>
    void getProperty(coil::Properties& prop, const char* key, T& value)
    {
    if (prop.findNode(key) != 0)
      {
        T tmp;
        if (coil::stringTo(tmp, prop[key].c_str()))
          {
            value = tmp;
          }
      }
    }
    OpenHRP::ExecutionProfileService::Profile m_profile;
    struct timeval m_tv;
    int m_priority;
  };
};

extern "C"
{
  void hrpExecutionContextInit(RTC::Manager* manager);
};

#endif // hrpEC_h

