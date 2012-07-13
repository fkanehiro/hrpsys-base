// -*- C++ -*-

#include "hrpEC.h"
#include <rtm/ECFactory.h>
#include "io/iob.h"

#include <iostream>

#include <stdio.h>
#include <errno.h>
#include <sched.h>
#include <signal.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <unistd.h>

#define MAX_SAFE_STACK (8*1024) /* The maximum stack size which is
                                   guranteed safe to access without
                                   faulting */
void stack_prefault(void) {

  unsigned char dummy[MAX_SAFE_STACK];

  memset(&dummy, 0, MAX_SAFE_STACK);
  return;
}

namespace RTC
{
    hrpExecutionContext::hrpExecutionContext()
        : PeriodicExecutionContext(), 
          m_priority(49)
    {
        resetProfile();
        rtclog.setName("hrpEC");
        coil::Properties& prop(Manager::instance().getConfig());
        
        // Priority
        getProperty(prop, "exec_cxt.periodic.priority", m_priority);
        getProperty(prop, "exec_cxt.periodic.rtpreempt.priority", m_priority);
        RTC_DEBUG(("Priority: %d", m_priority));
    }

    bool hrpExecutionContext::waitForNextPeriod()
    {
        if (wait_for_iob_signal()){
            perror("wait_for_iob_signal()");
            return false;
        }
        return true;
    }

    bool hrpExecutionContext::enterRT()
    {
        struct sched_param param;
        param.sched_priority = m_priority;

#ifndef __APPLE__
        if (sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
            perror("sched_setscheduler");
            std::cerr << "If you are running this program on normal linux kernel for debug purpose, you can ignore the error message displayed above. If not, this program must have superuser privilege." << std::endl;
            //return -1;
        }else{
            /* Lock memory */
            if(mlockall(MCL_CURRENT|MCL_FUTURE) == -1) {
                perror("mlockall failed");
            }
        }
#endif
        /* Pre-fault our stack */
        stack_prefault();

        return true;
    }
    bool hrpExecutionContext::exitRT()
    {
        return true;
    }
};


extern "C"
{
    void hrpECInit(RTC::Manager* manager)
    {
        manager->registerECFactory("hrpExecutionContext",
                                   RTC::ECCreate<RTC::hrpExecutionContext>,
                                   RTC::ECDelete<RTC::hrpExecutionContext>);
        std::cerr << "hrpExecutionContext is registered" << std::endl;
    }
};
 
