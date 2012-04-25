#ifndef __THREADED_OBJECT__
#define __THREADED_OBJECT__

#include <SDL/SDL_thread.h>

class ThreadedObject
{
public:
    ThreadedObject();
    ~ThreadedObject();
    void start();
    void stop();
    void pause();
    void resume();
    void wait();
    bool isPausing();
    bool isRunning();
    virtual bool oneStep();
private:
    bool m_isPausing, m_isRunning;
    SDL_Thread *m_thread;
    SDL_sem *m_sem;
};

#endif
