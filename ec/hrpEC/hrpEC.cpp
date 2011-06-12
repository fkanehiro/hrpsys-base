// -*- C++ -*-

#include "hrpEC.h"
#include <rtm/ECFactory.h>
#include "hrpIo/iob.h"

#include <iostream>

#ifdef USE_ART
#include <linux/art_task.h>
#else
#include <stdio.h>
#include <errno.h>
#include <sched.h>
#include <signal.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <unistd.h>

typedef short art_prio_t;
typedef unsigned short art_flags_t;
#define ART_TASK_PERIODIC  0x0001
#define ART_PRIO_MAX   255

static timespec g_ts;
static double g_period_ns;

#ifdef __darwin__
typedef int clockid_t;
#define CLOCK_MONOTONIC 0
#include <mach/mach_time.h>  
int clock_gettime(clockid_t clk_id, struct timespec *tp)
{
    if (clk_id != CLOCK_MONOTONIC) return -1;

    uint64_t clk;
    clk = mach_absolute_time();  

    static mach_timebase_info_data_t info = {0,0};  
    if (info.denom == 0) mach_timebase_info(&info);  
  
    uint64_t elapsednano = clk * (info.numer / info.denom);  
  
    tp->tv_sec = elapsednano * 1e-9;  
    tp->tv_nsec = elapsednano - (tp->tv_sec * 1e9);  
    return 0;
}

#define TIMER_ABSTIME 0
int clock_nanosleep(clockid_t clk_id, int flags, struct timespec *tp,
    struct timespec *remain)
{
    if (clk_id != CLOCK_MONOTONIC || flags != TIMER_ABSTIME) return -1;

    static mach_timebase_info_data_t info = {0,0};  
    if (info.denom == 0) mach_timebase_info(&info);  
  
    uint64_t clk = (tp->tv_sec*1e9 + tp->tv_nsec)/(info.numer/info.denom);
    
    mach_wait_until(clk);
    return 0;
}
#endif

void timespec_add_ns(timespec *ts, long ns)
{
    ts->tv_nsec += ns;
    while (ts->tv_nsec > 1e9){
        ts->tv_sec += 1;
        ts->tv_nsec -= 1e9;
    }
}

double timespec_compare(timespec *ts1, timespec *ts2)
{
    double dts = ts1->tv_sec - ts2->tv_sec;
    double dtn = ts1->tv_nsec - ts2->tv_nsec;
    return dts*1e9+dtn;
}

#define MAX_SAFE_STACK (8*1024) /* The maximum stack size which is
                                   guranteed safe to access without
                                   faulting */
void stack_prefault(void) {

  unsigned char dummy[MAX_SAFE_STACK];

  memset(&dummy, 0, MAX_SAFE_STACK);
  return;
}


int art_enter(art_prio_t prio, art_flags_t flag, unsigned long period)
{
    struct sched_param param;

    printf("ART API is emulated\n");
    if ((param.sched_priority = sched_get_priority_max(SCHED_FIFO)) == -1) {
	perror("sched_get_priority_max");
	return -1;
    }
    param.sched_priority -= ART_PRIO_MAX - prio;

#ifndef __APPLE__
    if (sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
	perror("sched_setscheduler");
	//return -1;
    }
#endif

    /* Lock memory */
    if(mlockall(MCL_CURRENT|MCL_FUTURE) == -1) {
      perror("mlockall failed");
    }

    /* Pre-fault our stack */
    stack_prefault();

    clock_gettime(CLOCK_MONOTONIC, &g_ts);
    g_period_ns = period*1e3;
    return 0;
}

int art_exit(void)
{
  return 0;
}

int art_wait(void)
{
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &g_ts, 0);
    timespec_add_ns(&g_ts, g_period_ns);
    timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    double dt = timespec_compare(&g_ts, &now);
    if (dt <= 0){
        //printf("overrun(%d[ms])\n", -dt*1e6);
        do {
            timespec_add_ns(&g_ts, g_period_ns);
        }while(timespec_compare(&g_ts, &now)<=0);
    }
    return 0;
}

#endif

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
#ifdef USE_ART
            while(1){
                if (art_wait() == -1){
                    perror("art_wait");
                    return 0;
                }
                if (read_iob_frame() % nsubstep == 0) break;
            }
#else
	    if (wait_for_iob_signal()){
	        perror("wait_for_iob_signal()");
	    }
#endif
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
 
