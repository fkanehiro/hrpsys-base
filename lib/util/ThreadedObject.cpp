#include "ThreadedObject.h"

static int threadMain(void *arg)
{
    ThreadedObject *throbj = (ThreadedObject *)arg;
    while(throbj->isRunning()){
        if (!throbj->oneStep()) break;
    }
}

ThreadedObject::ThreadedObject() : 
    m_thread(NULL), m_isPausing(false), m_isRunning(false)
{
    sem_init(&m_sem, 0, 0);
}

void ThreadedObject::pause(){
    m_isPausing = true;
}

void ThreadedObject::resume(){
    m_isPausing = false;
    sem_post(&m_sem);
}

bool ThreadedObject::isPausing(){
    return m_isPausing;
}

bool ThreadedObject::isRunning(){
    return m_isRunning;
}

bool ThreadedObject::oneStep(){
    if (m_isPausing){
        sem_wait(&m_sem); 
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
    m_isRunning = false;
    SDL_WaitThread(m_thread, NULL);
    m_thread = NULL;
}
