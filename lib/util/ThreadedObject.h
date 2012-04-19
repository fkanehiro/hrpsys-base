#ifndef __THREADED_OBJECT__
#define __THREADED_OBJECT__

#include <SDL/SDL_thread.h>
#include "semaphore.h"

class ThreadedObject
{
public:
    ThreadedObject();
    void start();
    void stop();
    void pause();
    void resume();
    bool isPausing();
    bool isRunning();
    virtual bool oneStep();
private:
    bool m_isPausing, m_isRunning;
    SDL_Thread *m_thread;
    sem_t m_sem;
};

#endif
