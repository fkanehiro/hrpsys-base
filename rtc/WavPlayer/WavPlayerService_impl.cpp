#include <iostream>
#include <cstdlib>
#include <cstdio>
#include <pthread.h>
#include "WavPlayerService_impl.h"

WavPlayerService_impl::WavPlayerService_impl()
{
}

WavPlayerService_impl::~WavPlayerService_impl()
{
}

void WavPlayerService_impl::playWav(const char *filename)
{
    char buf[256];
    sprintf(buf, "aplay %s", filename);
    std::cout << "cmd = [" << buf << "]" << std::endl;
    system(buf);
}

void *thread_main(void *args)
{
    char *filename = (char *)args;
    char buf[256];
    sprintf(buf, "aplay %s", filename);
    std::cout << "cmd = [" << buf << "]" << std::endl;
    system(buf);
    return NULL;
}

void WavPlayerService_impl::playWavNoWait(const char *filename)
{
    pthread_t thr;
    pthread_create(&thr, NULL, thread_main, (void *)filename);
}
