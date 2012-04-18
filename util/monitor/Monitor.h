#include <string>
#include <SDL_thread.h>
#include "StateHolderService.hh"
#include "TimedRobotState.h"

class Monitor
{
public:
    Monitor(CORBA::ORB_var orb, const std::string &i_hostname,
            int i_port, int i_interval);
    void start();
    void stop();
    bool run();
    bool isConnected();
private:
    SDL_Thread *m_thread;
    CORBA::ORB_var m_orb;
    CosNaming::NamingContext_var m_naming;
    bool m_flagExit;
    std::string m_rhCompName, m_shCompName;
    OpenHRP::RobotHardwareService_var m_rhService;
    OpenHRP::StateHolderService_var   m_shService;
    TimedRobotState m_rstate;
    int m_interval;
};
