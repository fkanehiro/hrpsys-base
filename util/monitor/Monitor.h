#include <string>
#include "util/ThreadedObject.h"
#include "StateHolderService.hh"
#include "TimedRobotState.h"

class Monitor : public ThreadedObject
{
public:
    Monitor(CORBA::ORB_var orb, const std::string &i_hostname,
            int i_port, int i_interval);
    bool oneStep();
    bool isConnected();
private:
    CORBA::ORB_var m_orb;
    CosNaming::NamingContext_var m_naming;
    std::string m_rhCompName, m_shCompName;
    OpenHRP::RobotHardwareService_var m_rhService;
    OpenHRP::StateHolderService_var   m_shService;
    TimedRobotState m_rstate;
    int m_interval;
};
