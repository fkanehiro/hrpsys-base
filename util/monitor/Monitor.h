#include <string>
#include "util/ThreadedObject.h"
#include "util/LogManager.h"
#include "StateHolderService.hh"
#include "TimedRobotState.h"

class Monitor : public ThreadedObject
{
public:
    Monitor(CORBA::ORB_var orb, const std::string &i_hostname,
            int i_port, int i_interval, LogManager<TimedRobotState> *i_log);
    bool oneStep();
    bool isConnected();
    void setRobotHardwareName(const char *i_name);
    void setStateHolderName(const char *i_name);
private:
    CORBA::ORB_var m_orb;
    CosNaming::NamingContext_var m_naming;
    std::string m_rhCompName, m_shCompName;
    OpenHRP::RobotHardwareService_var m_rhService;
    OpenHRP::StateHolderService_var   m_shService;
    TimedRobotState m_rstate;
    LogManager<TimedRobotState> *m_log;
    int m_interval;
};
