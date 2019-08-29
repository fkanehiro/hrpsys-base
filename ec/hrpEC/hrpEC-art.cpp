// -*- C++ -*-
#include <iostream>
#include "hrpEC.h"
#include <rtm/ECFactory.h>
#include "hrpsys/io/iob.h"
#include <linux/art_task.h>

namespace RTC
{
    hrpExecutionContext::hrpExecutionContext()
        : PeriodicExecutionContext(), 
          m_priority(ART_PRIO_MAX-1),
          m_thread_pending (false)
    {
        resetProfile();
        rtclog.setName("hrpEC");
        coil::Properties& prop(Manager::instance().getConfig());
        
        // Priority
        getProperty(prop, "exec_cxt.periodic.priority", m_priority);
        getProperty(prop, "exec_cxt.periodic.art.priority", m_priority);
        RTC_DEBUG(("Priority: %d", m_priority));
    }

    bool hrpExecutionContext::waitForNextPeriod()
    {
	int nsubstep = number_of_substeps();
        while(1){
            if (art_wait() == -1){
                perror("art_wait");
                return false;
            }
            if (read_iob_frame() % nsubstep == 0) break;
        }
        return true;
    }
    bool hrpExecutionContext::enterRT()
    {
        unsigned long period_usec 
            = (m_period.sec()*1e6+m_period.usec())/number_of_substeps();
        if (art_enter(m_priority, ART_TASK_PERIODIC, period_usec) == -1){
            perror("art_enter");
            return false;
        }
        return true;
    }
    bool hrpExecutionContext::exitRT()
    {
        if (art_exit() == -1){
            perror("art_exit");
            return false;
        }
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
        std::cerr << "hrpExecutionContext for ART is registered" << std::endl;
    }
};
 
