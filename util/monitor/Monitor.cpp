#include <rtm/CorbaNaming.h>
#include "Monitor.h"
#include "util/OpenRTMUtil.h"
#include "GLscene.h"

Monitor::Monitor(CORBA::ORB_var orb, const std::string &i_hostname,
                 int i_port, int i_interval, LogManager<TimedRobotState> *i_log) :
    m_orb(orb),
    m_rhCompName("RobotHardware0"),
    m_shCompName("StateHolder0"),
    m_interval(i_interval),
    m_log(i_log)
{
    char buf[128];
    try {
        sprintf(buf, "%s:%d", i_hostname.c_str(), i_port);
        RTC::CorbaNaming naming(orb, buf);
        m_naming = CosNaming::NamingContext::_duplicate(naming.getRootContext());
    }catch (CORBA::SystemException& ex) {
        std::cerr << "[monitor] Failed to initialize CORBA " << std::endl << ex._rep_id() << " with " << buf << std::endl;
    }catch (const std::string& error){
        std::cerr << "[monitor] Failed to initialize CORBA " << std::endl << error << " with " << buf << std::endl;
    }catch (...){
        std::cerr << "[monitor] Failed to initialize CORBA with " << buf << std::endl;
    }
}

bool Monitor::oneStep()
{
    static long long loop = 0;
    ThreadedObject::oneStep();

    // RobotHardwareService
    if (CORBA::is_nil(m_rhService)){
        try{
            CosNaming::Name name;
            name.length(1);
            name[0].id = CORBA::string_dup(m_rhCompName.c_str());
            name[0].kind = CORBA::string_dup("rtc");
            CORBA::Object_var obj = m_naming->resolve(name);
            RTC::RTObject_var rtc = RTC::RTObject::_narrow(obj);
            RTC::ExecutionContextList_var eclist = rtc->get_owned_contexts();
            for(CORBA::ULong i=0; i < eclist->length(); ++i){
                eclist[i]->activate_component(rtc);
            }
            const char *ior = getServiceIOR(rtc, "RobotHardwareService");
            m_rhService = OpenHRP::RobotHardwareService::_narrow(m_orb->string_to_object(ior));
        }catch(...){
            if ( (loop%(5*(1000/m_interval))) == 0 )
                std::cerr << "[monitor] RobotHardwareService could not connect (" << m_rhCompName << ")" << std::endl;
        }
    } else {
        if ( (loop%(5*(1000/m_interval))) == 0 )
            std::cerr << "[monitor] RobotHardwareService is not found (" << m_rhCompName << ")" << std::endl;
    }
    // StateHolderService
    if (CORBA::is_nil(m_shService)){
        try{
            CosNaming::Name name;
            name.length(1);
            name[0].id = CORBA::string_dup(m_shCompName.c_str());
            name[0].kind = CORBA::string_dup("rtc");
            CORBA::Object_var obj = m_naming->resolve(name);
            RTC::RTObject_var rtc = RTC::RTObject::_narrow(obj);
            const char *ior = getServiceIOR(rtc, "StateHolderService");
            m_shService = OpenHRP::StateHolderService::_narrow(m_orb->string_to_object(ior));
        }catch(...){
            if ( (loop%(5*(1000/m_interval))) == 0 )
            std::cerr << "[monitor] StateHolderService could not connect (" << m_shCompName << ")" << std::endl;
        }
    }else{
        if ( (loop%(5*(1000/m_interval))) == 0 )
            std::cerr << "[monitor] StateHolderService is not found (" << m_shCompName << ")" << std::endl;
    }

    bool stateUpdate = false;
    if (!CORBA::is_nil(m_rhService)){
        OpenHRP::RobotHardwareService::RobotState_var rs;
        try{
            m_rhService->getStatus(rs);
            m_rstate.state = rs;
            stateUpdate = true;
        }catch(...){
            std::cerr << "[monitor] exception in getStatus()" << std::endl;
            m_rhService = NULL;
        }
    }

    if (!CORBA::is_nil(m_shService)){
        OpenHRP::StateHolderService::Command_var com;
        try{
            m_shService->getCommand(com); 
            m_rstate.command = com;
            stateUpdate = true;
        }catch(...){
            std::cerr << "[monitor] exception in getCommand()" << std::endl;
            m_shService = NULL;
        }
    }


    if (stateUpdate) {
        struct timeval tv;
        gettimeofday(&tv, NULL);
        m_rstate.time = tv.tv_sec + tv.tv_usec/1e6; 
        m_log->add(m_rstate);
    }
    usleep(1000*m_interval);
    loop ++;

    return true;
}

bool Monitor::isConnected()
{
    return !CORBA::is_nil(m_rhService);
}

extern bool isCalibrated(int s);
extern bool isPowerOn(int s);
extern bool isServoOn(int s);
extern int servoAlarm(int s);
extern int temperature(int s);

void Monitor::showStatus(hrp::BodyPtr &body)
{
    if (m_log->index()<0) return;

    LogManager<TimedRobotState> *lm
        = (LogManager<TimedRobotState> *)m_log;
    OpenHRP::RobotHardwareService::RobotState &rstate = lm->state().state;

    //fprintf(stdout, "\e[2J\e[1;1H"); // clear + home
    fprintf(stdout, "\e[1;1H"); // home
    fprintf(stdout, "\x1b[42m"); // greep backgroupd
    //fprintf(stdout, "\e[2K"); // clear 1 line
    fprintf(stdout, "\e[2KID PW                 NAME    ANGLE  COMMAND TORQUE SERVO TEMP\n"); // greep backgroupd
    char buf[256];
    for (int i=0; i<body->numJoints(); i++){
        hrp::Link *l = body->joint(i);
        if (l){
            fprintf(stdout,"\e[2K");
            int ss = rstate.servoState[i][0];
            // joint ID
            if (!isCalibrated(ss)){
                yellow();
            }else if(isServoOn(ss)){
                red();
            }
            fprintf(stdout, "%2d ",i);
            black();
            // power status
            if (isPowerOn(ss)) blue();
            fprintf(stdout, " o ");
            if (isPowerOn(ss)) black();
            // joint name, current angle, command angle and torque
            fprintf(stdout, "%20s %8.3f %8.3f %6.1f   ",
                    l->name.c_str(),
                    (i<rstate.angle.length())?(rstate.angle[i]*180/M_PI):0,
                    (i<rstate.command.length())?(rstate.command[i]*180/M_PI):0,
                    (i<rstate.torque.length())?(rstate.torque[i]*180/M_PI):0);
            // servo alarms
            fprintf(stdout, "%03x   ", servoAlarm(ss));
            // driver temperature
            int temp = temperature(ss);
            if (!temp){
                fprintf(stdout, "-- ", temp);
            }else{
                if (temp >= 60) red();
                fprintf(stdout, "%2d ", temp);
                if (temp >= 60) black();
            }
            fprintf(stdout, "\n");
        }
    }
    fprintf(stdout, "\e[2K---\n");

    if (rstate.accel.length()){
        fprintf(stdout, "\e[2K         acc:");
        for (unsigned int i=0; i<rstate.accel.length(); i++){
            if(i>0)fprintf(stdout, "\e[2K             ");
            fprintf(stdout, " %8.4f %8.4f %8.4f\n",
                    rstate.accel[i][0], rstate.accel[i][1], rstate.accel[i][2]);
        }
    }
    if (rstate.rateGyro.length()){
        fprintf(stdout, "\e[2K        rate:");
        for (unsigned int i=0; i<rstate.rateGyro.length(); i++){
            if(i>0)fprintf(stdout, "\e[2K             ");
            fprintf(stdout, " %8.4f %8.4f %8.4f\n",
                    rstate.rateGyro[i][0], rstate.rateGyro[i][1], rstate.rateGyro[i][2]);
        }
    }
    if (rstate.force.length()){
        fprintf(stdout, "\e[2Kforce/torque:");
        for (unsigned int i=0; i<rstate.force.length(); i++){
            if(i>0)fprintf(stdout, "\e[2K             ");
            fprintf(stdout, " %6.1f %6.1f %6.1f %6.2f %6.2f %6.2f\n",
                    rstate.force[i][0],
                    rstate.force[i][1],
                    rstate.force[i][2],
                    rstate.force[i][3],
                    rstate.force[i][4],
                    rstate.force[i][5]);
            }
    }
    fprintf(stdout, "\e[2K\n");
}


void Monitor::setRobotHardwareName(const char *i_name)
{
    m_rhCompName = i_name;
}

void Monitor::setStateHolderName(const char *i_name)
{
    m_shCompName = i_name;
}
