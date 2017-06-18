#ifndef TIME_UTIL_H
#define TIME_UTIL_H

/**
   \brief time stamp counter
*/
typedef unsigned long long tick_t;

/**
   \brief get time stamp counter
   \return value of time stamp counter
*/
tick_t get_tick();

/**
   \brief get CPU frequency
   \return CPU frequency[kHz]
*/
double get_cpu_frequency();

/**
   \brief convert time stamp counter into usec
   \param t value of time stamp counter
*/
#define tick2usec(t) ((t)*1e6 / get_cpu_frequency())

/**
   \brief convert time stamp counter into msec
   \param t value of time stamp counter
*/
#define tick2msec(t) ((t)*1e3 / get_cpu_frequency())

/**
   \brief convert time stamp counter into sec
   \param t value of time stamp counter
*/
#define tick2sec(t) ((t) / get_cpu_frequency())

#endif
