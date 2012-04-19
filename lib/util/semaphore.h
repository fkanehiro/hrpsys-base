#ifndef __HRPSYS_UTIL_SEMAPHORE_H__
#define __HRPSYS_UTIL_SEMAPHORE_H__

#ifdef __APPLE__
typedef struct{
    pthread_cond_t cond;
    pthread_mutex_t mutex;
    int count;
} unnamed_sem_t;
#define sem_t unnamed_sem_t
#define sem_init(x,y,z) { pthread_cond_init(&((x)->cond), NULL); pthread_mutex_init(&((x)->mutex), NULL); (x)->count = z;}
#define sem_wait(x) { pthread_mutex_lock(&((x)->mutex)); if ((x)->count <= 0) pthread_cond_wait(&((x)->cond), &((x)->mutex)); (x)->count--; pthread_mutex_unlock(&((x)->mutex)); }
#define sem_post(x) { pthread_mutex_lock(&((x)->mutex)); (x)->count++; pthread_cond_signal(&((x)->cond)); pthread_mutex_unlock(&((x)->mutex)); }
#else
#include <semaphore.h>
#endif

#endif
