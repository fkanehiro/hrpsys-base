// -*- C++ -*-
#include <iostream>
#include "hrpEC.h"
#include <rtm/ECFactory.h>
#include "io/iob.h"
#include <linux/art_task.h>

namespace RTC
{
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
        double period_usec = m_period.sec()*1e6+m_period.usec();
        if (art_enter(ART_PRIO_MAX-1, ART_TASK_PERIODIC, period_usec) == -1){
            perror("art_enter");
            close_iob();
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
 
