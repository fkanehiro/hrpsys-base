#include "Monitor.h"

static int threadMain(void *arg)
{
    Monitor *monitor = (Monitor *)arg;
    while(monitor->run());
}

Monitor::Monitor() :
    m_flagExit(false),
    m_rhCompName("RobotHardware0"),
    m_shCompName("StateHolder0")
{
}

void Monitor::start()
{
    m_thread = SDL_CreateThread(threadMain, NULL);
}

void Monitor::stop()
{
    m_flagExit = true;
    SDL_WaitThread(m_thread, NULL);
}

bool Monitor::run()
{
    return !m_flagExit;
}
