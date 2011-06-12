#ifndef __OS_WRAPPER_H__
#define __OS_WRAPPER_H__

#include <pthread.h>
#ifdef __APPLE__
typedef struct{
    pthread_cond_t cond;
    pthread_mutex_t mutex;
} unnamed_sem_t;
#define sem_t unnamed_sem_t
#define sem_init(x,y,z) { pthread_cond_init(&((x)->cond), NULL); pthread_mutex_init(&((x)->mutex), NULL); }
#define sem_wait(x) { pthread_mutex_lock(&((x)->mutex)); pthread_cond_wait(&((x)->cond), &((x)->mutex)); pthread_mutex_unlock(&((x)->mutex)); }
#define sem_post(x) { pthread_mutex_lock(&((x)->mutex)); pthread_cond_signal(&((x)->cond)); pthread_mutex_unlock(&((x)->mutex)); }
#else
#include <semaphore.h>
#endif

#endif
