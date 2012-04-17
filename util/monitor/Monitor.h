#include <string>
#include <SDL_thread.h>
#include "RobotHardwareService.hh"
#include "StateHolderService.hh"

class Monitor
{
public:
    Monitor();
    void start();
    void stop();
    bool run();
private:
    SDL_Thread *m_thread;
    bool m_flagExit;
    std::string m_rhCompName, m_shCompName;
    OpenHRP::RobotHardwareService_var m_rhService;
    OpenHRP::StateHolderService_var   m_shService;
};
