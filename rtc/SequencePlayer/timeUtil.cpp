#include <fstream>
#include "hrpsys/util/Hrpsys.h"
#include "timeUtil.h"
#include <stdint.h>


tick_t get_tick()
{
#ifdef _WIN32
    LARGE_INTEGER t;
    QueryPerformanceCounter(&t);
    return t.QuadPart;
#elif defined(__ARM_ARCH_7A__)
    uint32_t r = 0;
    asm volatile("mrc p15, 0, %0, c9, c13, 0" : "=r"(r) );
    return r;
#elif defined(__AARCH64EL__)
    uint64_t b;
    asm volatile( "mrs %0, pmccntr_el0" : "=r"(b) :: "memory" );
    return b;
#else
    unsigned int l=0,h=0;
    __asm__ __volatile__("rdtsc" : "=a" (l), "=d" (h));
    return (unsigned long long)h<<32|l;
#endif
}

double get_cpu_frequency()
{
    static double freq = -1;
    if (freq != -1) return freq;
#ifdef _WIN32
    LARGE_INTEGER li;
    QueryPerformanceFrequency(&li);
    freq = (double)li.QuadPart;
#else
# ifdef __QNX__
    freq = SYSPAGE_ENTRY( qtime )->cycles_per_sec;
# else
    std::ifstream ifs("/proc/cpuinfo");
    std::string token;
    while(!ifs.eof()){
        ifs >> token;
        if (token == "cpu"){
            ifs >> token;
            if (token == "MHz"){
                ifs >> token;
                ifs >> freq;
                freq *= 1e6;
                break;
            }
        }
    }
    ifs.close();
# endif
#endif
    return freq;
}
