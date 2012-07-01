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

namespace RTC
{
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
        std::cerr << "hrpExecutionContext is registered" << std::endl;
    }
};
 
