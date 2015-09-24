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
#ifndef OPENRTM_VERSION_TRUNK
      : public virtual PeriodicExecutionContext,
#else
      : public virtual RTC_exp::PeriodicExecutionContext,
#endif
        public virtual POA_OpenHRP::ExecutionProfileService,
        public virtual PortableServer::RefCountServantBase
  {
  public:
    hrpExecutionContext();
    virtual ~hrpExecutionContext();
    virtual int svc(void);
#ifdef OPENRTM_VERSION_TRUNK
    virtual void tick(){}
#endif

    OpenHRP::ExecutionProfileService::Profile *getProfile();
    OpenHRP::ExecutionProfileService::ComponentProfile getComponentProfile(RTC::LightweightRTObject_ptr obj);
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
    std::vector<std::string> rtc_names;
  };
};

extern "C"
{
  void hrpExecutionContextInit(RTC::Manager* manager);
};

#endif // hrpEC_h

