#ifndef BODY_EXT_H_INCLUDED
#define BODY_EXT_H_INCLUDED

#include <rtm/idl/BasicDataType.hh>
#include <rtm/idl/ExtendedDataTypes.hh>
#include "hrpsys/idl/Img.hh"
#include "hrpsys/idl/RobotHardwareService.hh"
#include <hrpModel/Body.h>
#include <hrpCorba/OpenHRPCommon.hh>
#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>
#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/idl/ExtendedDataTypesSkel.h>

class InPortHandlerBase;
class OutPortHandlerBase;
class EmergencySignalPortHandler;
class BodyRTC;

class RobotHardwareServicePort
    : public virtual POA_OpenHRP::RobotHardwareService,
      public virtual PortableServer::RefCountServantBase
{
public:
    RobotHardwareServicePort();
    ~RobotHardwareServicePort();
    void getStatus(OpenHRP::RobotHardwareService::RobotState_out rs);
    void getStatus2(OpenHRP::RobotHardwareService::RobotState2_out rs);
    CORBA::Boolean power(const char* jname, OpenHRP::RobotHardwareService::SwitchStatus ss);
    CORBA::Boolean servo(const char* jname, OpenHRP::RobotHardwareService::SwitchStatus ss);
    void setServoGainPercentage(const char *jname, double limit);
    void setServoErrorLimit(const char *jname, double limit);
    void calibrateInertiaSensor();
    void removeForceSensorOffset();
    void initializeJointAngle(const char* name, const char* option);
    CORBA::Boolean addJointGroup(const char* gname, const OpenHRP::RobotHardwareService::StrSequence& jnames);
    CORBA::Boolean readDigitalInput(::OpenHRP::RobotHardwareService::OctSequence_out din);
    CORBA::Long lengthDigitalInput();
    CORBA::Boolean writeDigitalOutput(const ::OpenHRP::RobotHardwareService::OctSequence& dout);
    CORBA::Boolean writeDigitalOutputWithMask(const ::OpenHRP::RobotHardwareService::OctSequence& dout, const ::OpenHRP::RobotHardwareService::OctSequence& mask);
    CORBA::Long lengthDigitalOutput();
    CORBA::Boolean readDigitalOutput(::OpenHRP::RobotHardwareService::OctSequence_out dout);
    void setRobot(BodyRTC *i_robot);
private:
    BodyRTC *m_robot;
};

//

class BodyRTC : virtual public hrp::Body, public RTC::DataFlowComponentBase
{
public:
    BodyRTC(RTC::Manager* manager = &RTC::Manager::instance());
    virtual ~BodyRTC(void);

    RTC::ReturnCode_t setup();

    RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id){
        std::cout << "BodyRTC::onActivated(" << ec_id << ")" << std::endl;
        return RTC::RTC_OK;
    }
    RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id){
        std::cout << "BodyRTC::onDeactivated(" << ec_id << ")" << std::endl;
        return RTC::RTC_OK;
    }

    void createInPort(const std::string &config);
    void createOutPort(const std::string &config);
    void writeDataPorts(double time);
    void readDataPorts();
    static void moduleInit(RTC::Manager*);

    void getStatus(OpenHRP::RobotHardwareService::RobotState* rs);
    void getStatus2(OpenHRP::RobotHardwareService::RobotState2* rs);

    bool preOneStep();
    bool postOneStep();
    bool names2ids(const std::vector<std::string> &i_names, std::vector<int> &o_ids);
    std::vector<int> getJointGroup(const char* gname) {return  m_jointGroups[gname]; }
    int addJointGroup(const char* gname, const std::vector<int>jids) { m_jointGroups[gname] = jids; }

    // API to be compatible with robot.h
    bool power(int jid, bool turnon) {power_status[jid] = turnon?OpenHRP::RobotHardwareService::SWITCH_ON:OpenHRP::RobotHardwareService::SWITCH_OFF; }
    bool servo(const char *jname, bool turnon);
    bool servo(int jid, bool turnon) {servo_status[jid] = turnon?OpenHRP::RobotHardwareService::SWITCH_ON:OpenHRP::RobotHardwareService::SWITCH_OFF; }
    bool power(const char *jname, bool turnon);

    //void removeForceSensorOffset();
    //bool loadGain();
    //void startInertiaSensorCalibration();
    //void startForceSensorCalibration();
    //void initializeJointAngle(const char *name, const char *option);
    //void oneStep();
    int readCalibState(const int i) { return (calib_status[i] == OpenHRP::RobotHardwareService::SWITCH_ON); }
    int readPowerState(const int i) { return (power_status[i] == OpenHRP::RobotHardwareService::SWITCH_ON); }
    int readServoState(const int i) { return (servo_status[i] == OpenHRP::RobotHardwareService::SWITCH_ON); }
    //int readServoAlarm(int i);
    //int readDriverTemperature(int i);
    //void readPowerStatus(double &o_voltage, double &o_current);
    //void readJointAngles(double *o_angles);
    //void readJointVelocities(double *o_velocities);
    //int readJointTorques(double *o_torques);
    //void readGyroSensor(unsigned int i_rank, double *o_rates);
    //void readAccelerometer(unsigned int i_rank, double *o_accs);
    //void readForceSensor(unsigned int i_rank, double *o_forces);
    //void writeJointCommands(const double *i_commands);
    //void readJointCommands(double *o_commands);
    //void writeTorqueCommands(const double *i_commands);
    //void writeVelocityCommands(const double *i_commands);
    //size_t lengthOfExtraServoState(int id);
    //void readExtraServoState(int id, int *state);
    typedef enum {EMG_NONE, EMG_SERVO_ERROR, EMG_FZ} emg_reason;
    bool checkEmergency(emg_reason &o_reason, int &o_id);
    //bool setServoGainPercentage(const char *i_jname, double i_percentage);
    bool setServoErrorLimit(const char *i_jname, double i_limit);
    //void setProperty(const char *key, const char *value);
    //bool addJointGroup(const char *gname, const std::vector<std::string>& jnames);
    std::vector<double> m_servoErrorLimit;
    //double m_fzLimitRatio;
    //double m_maxZmpError;

    bool readDigitalInput(char *o_din);
    int lengthDigitalInput();
    bool writeDigitalOutput(const char *i_dout);
    bool writeDigitalOutputWithMask(const char *i_dout, const char *i_mask);
    int lengthDigitalOutput();
    bool readDigitalOutput(char *o_dout);

    bool resetPosition() { m_resetPosition = true; }
    //
    BodyRTC::emg_reason m_emergencyReason;
    int m_emergencyId;

private:
    static const char* bodyrtc_spec[];
    // DataInPort
    std::vector<InPortHandlerBase *> m_inports;

    // DataOutPort
    std::vector<OutPortHandlerBase *> m_outports;

    // Corba Port
    RTC::CorbaPort m_RobotHardwareServicePort;
    RobotHardwareServicePort m_service0;

    // robot status
    std::vector<double> angles;
    std::vector<double> commands;
    std::vector<hrp::Vector3> accels;
    std::vector<hrp::Vector3> gyros;
    std::vector<hrp::dvector6> forces;
    std::vector<OpenHRP::RobotHardwareService::SwitchStatus> calib_status;
    std::vector<OpenHRP::RobotHardwareService::SwitchStatus> servo_status;
    std::vector<OpenHRP::RobotHardwareService::SwitchStatus> power_status;
    std::map<std::string, std::vector<int> > m_jointGroups;

    // pinned position for servo off
    bool m_resetPosition;
    hrp::Vector3  m_lastServoOn_p;
    hrp::Matrix33 m_lastServoOn_R;

    int dummy;
};

typedef boost::intrusive_ptr<BodyRTC> BodyRTCPtr;

#endif
