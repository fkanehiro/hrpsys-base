#include <string>
#include "hrpsys/util/ThreadedObject.h"
#include "hrpsys/util/LogManager.h"
#include "hrpsys/idl/StateHolderService.hh"
#include "TimedRobotState.h"
#include "hrpModel/Body.h"

class Monitor : public ThreadedObject
{
public:
    Monitor(CORBA::ORB_var orb, const std::string &i_hostname,
            int i_port, int i_interval, LogManager<TimedRobotState> *i_log);
    bool oneStep();
    void showStatus(hrp::BodyPtr &body);
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

    void white()  { fprintf(stdout, "\x1b[37m");}
    void red()    { fprintf(stdout, "\x1b[31m");}
    void yellow() { fprintf(stdout, "\x1b[33m");}
    void green()  { fprintf(stdout, "\x1b[32m");}
    void blue()   { fprintf(stdout, "\x1b[34m");}
    void magenta(){ fprintf(stdout, "\x1b[35m");}
    void black()  { fprintf(stdout, "\x1b[30m");}

};
