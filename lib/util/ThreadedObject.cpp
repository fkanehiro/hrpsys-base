#include "ThreadedObject.h"

static int threadMain(void *arg)
{
    ThreadedObject *throbj = (ThreadedObject *)arg;
    while(throbj->isRunning()){
        if (!throbj->oneStep()) break;
    }
    throbj->notifyFinish();
}

ThreadedObject::ThreadedObject() : 
    m_isPausing(false), m_isRunning(false), m_thread(NULL)
{
    m_sem = SDL_CreateSemaphore(0);
}

ThreadedObject::~ThreadedObject()
{
    SDL_DestroySemaphore(m_sem);
}

void ThreadedObject::pause(){
    m_isPausing = true;
}

void ThreadedObject::resume(){
    m_isPausing = false;
    SDL_SemPost(m_sem);
}

bool ThreadedObject::isPausing(){
    return m_isPausing;
}

bool ThreadedObject::isRunning(){
    return m_isRunning;
}

bool ThreadedObject::oneStep(){
    if (m_isPausing){
        SDL_SemWait(m_sem); 
    }
    return true;
}

void ThreadedObject::start()
{
    if (m_thread) return;
    m_isRunning = true;
    m_thread = SDL_CreateThread(threadMain, (void *)this);
}

void ThreadedObject::stop()
{
    if (m_isPausing) resume();
    m_isRunning = false;
    wait();
}

void ThreadedObject::wait()
{
    SDL_WaitThread(m_thread, NULL);
    m_thread = NULL;
}

void ThreadedObject::notifyFinish()
{
    m_isRunning = false;
}
