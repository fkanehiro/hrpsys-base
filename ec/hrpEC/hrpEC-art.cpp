// -*- C++ -*-

#include "hrpEC.h"
#include <rtm/ECFactory.h>
#include "io/iob.h"

#include <iostream>

#include <linux/art_task.h>

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
        : PeriodicExecutionContext()
    {
        resetProfile();
    }

    hrpExecutionContext::~hrpExecutionContext()
    {
        close_iob();
    }


    int hrpExecutionContext::svc(void)
    {
        if (open_iob() == FALSE){
            std::cerr << "open_iob: failed to open" << std::endl;
            return 0;
        } 
        if (lock_iob() == FALSE){
            std::cerr << "failed to lock iob" << std::endl;
            close_iob();
            return 0;
        }
        double period_sec = (m_period.sec()+m_period.usec()/1e6);
	int nsubstep = number_of_substeps();
        std::cout << "period = " << period_sec*1e3*nsubstep << "[ms]" << std::endl;
        if (art_enter(ART_PRIO_MAX-1, ART_TASK_PERIODIC, period_sec*1e6) == -1){
            perror("art_enter");
            close_iob();
            return 0;
        }
        do{
            while(1){
                if (art_wait() == -1){
                    perror("art_wait");
                    return 0;
                }
                if (read_iob_frame() % nsubstep == 0) break;
            }
            struct timeval tv;
            gettimeofday(&tv, NULL);
            if (m_profile.count > 0){
#define DELTA_SEC(start, end) end.tv_sec - start.tv_sec + (end.tv_usec - start.tv_usec)/1e6;
                double dt = DELTA_SEC(m_tv, tv);
                if (dt > m_profile.max_period) m_profile.max_period = dt;
                if (dt < m_profile.min_period) m_profile.min_period = dt;
                m_profile.avg_period = (m_profile.avg_period*m_profile.count + dt)/(m_profile.count+1);
            }
            m_profile.count++;
            m_tv = tv;

            invoke_worker iw;
            struct timeval tbegin, tend;
	    std::vector<double> processes(m_comps.size());
            gettimeofday(&tbegin, NULL);
            for (unsigned int i=0; i< m_comps.size(); i++){
                iw(m_comps[i]);
                gettimeofday(&tend, NULL);
                double dt = DELTA_SEC(tbegin, tend);
                processes[i] = dt;
                tbegin = tend;
            }

            gettimeofday(&tv, NULL);
            double dt = DELTA_SEC(m_tv, tv);
            if (dt > m_profile.max_total_process) m_profile.max_total_process = dt;
	    if (m_profile.max_processes.length() != processes.size()){
	        m_profile.max_processes.length(processes.size());
		for (unsigned int i=0; i<m_profile.max_processes.length(); i++){
		    m_profile.max_processes[i] = 0.0;
		}
	    }
	    for (unsigned int i=0; i<m_profile.max_processes.length(); i++){
	        if (m_profile.max_processes[i] < processes[i]){
		    m_profile.max_processes[i] = processes[i];
		}
	    }
            if (dt > period_sec*nsubstep){
  	        m_profile.timeover++; 
#ifdef NDEBUG
                fprintf(stderr, "Timeover: processing time = %4.1f[ms]\n", dt*1e3);
                for (unsigned int i=0; i< processes.size(); i++){
                    fprintf(stderr, "%4.1f, ", processes[i]*1e3);
                }
                fprintf(stderr, "\n");
#endif
            }

        } while (m_running);
        if (art_exit() == -1){
            perror("art_exit");
            return 0;
        }
        unlock_iob();
        close_iob();

        return 0;
    }

    OpenHRP::ExecutionProfileService::Profile *hrpExecutionContext::getProfile()
    {
        OpenHRP::ExecutionProfileService::Profile *ret 
            = new OpenHRP::ExecutionProfileService::Profile;
        *ret = m_profile;
        return ret;
    }

    void hrpExecutionContext::resetProfile()
    {
        m_profile.max_period = m_profile.avg_period = 0;
        m_profile.min_period = 1.0; // enough long 
        m_profile.max_total_process = 0;
	for( unsigned int i = 0 ; i < m_profile.max_processes.length() ; i++ )
	    m_profile.max_processes[i] = 0;
        m_profile.count = m_profile.timeover = 0;
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
 
