#ifndef __ROBOT_H__
#define __ROBOT_H__

#include <boost/array.hpp>
#include <boost/interprocess/sync/interprocess_semaphore.hpp>
#include <hrpModel/Body.h>

/**
   \brief 
 */
class robot : public hrp::Body
{
public:
    /**
       \brief constructor
     */
    robot();

    /**
       \brief
     */
    bool init();

    /**
       \brief turn on/off joint servo
       \param jid joint id of the joint
       \param turnon true to turn on joint servo, false otherwise 
     */
    bool servo(int jid, bool turnon);

    /**
       \brief turn on/off joint servo
       \param jname name of the joint
       \param turnon true to turn on joint servo, false otherwise 
     */
    bool servo(const char *jname, bool turnon);

    /**
       \brief turn on/off power for joint servo
       \param jid joint id of the joint
       \param turnon true to turn on power, false otherwise 
     */
    bool power(int jid, bool turnon);

    /**
       \brief turn on/off power for joint servo
       \param jname name of the joint
       \param turnon true to turn on power, false otherwise 
     */
    bool power(const char *jname, bool turnon);

    /**
       \brief remove offsets on force sensor outputs
     */
    void removeForceSensorOffset();

    /**
       \brief load PD gains 
       \param fname name of the file where gains are stored
       \return true if gains are loaded successufully, false otherwise
     */
    bool loadGain(const char *fname = "PDgains.sav");

    /**
       \brief start inertia sensor calibration and wait until finish
    */
    void startInertiaSensorCalibration();

    /**
       \brief initialize joint angle
       \param name joint name, part name or all
       \param option options for initialization
     */
    void initializeJointAngle(const char *name, const char *option);

    /**
       \brief all processings for one sampling period
     */
    void oneStep();

    /**
       \brief read calibration status of a joint servo
       \param i joint id
       \return ON if the joint is already calibrated successfully, OFF otherwise
     */
    int readCalibState(int i);

    /**
       \brief read power status of a joint servo
       \param i joint id
       \return ON if power for the joint servo is on, OFF otherwise
     */
    int readPowerState(int i);

    /**
       \brief read servo status of a joint servo
       \param i joint id
       \return ON if the joint servo is on, OFF otherwise
     */
    int readServoState(int i);

    /**
       \brief read alarm information of a joint servo
       \param i joint id
       \return 0 if there is no alarm, see iob.h for more details.
     */
    int readServoAlarm(int i);

    /**
       \brief read temperature of motor driver
       \param i joint id
       \return 0 if temperature can't be measured
     */
    int readDriverTemperature(int i);

    /**
       \brief read voltage and current of the robot power source
       \param o_voltage voltage
       \param o_current current
     */
    void readPowerStatus(double &o_voltage, double &o_current);

    /**
       \brief read array of all joint angles[rad]
       \param o_angles array of all joint angles
     */
    void readJointAngles(double *o_angles);

    /**
       \brief read array of all joint torques[Nm]
       \param o_torques array of all joint torques
       \param TRUE if read successfully, FALSE otherwise
     */
    int readJointTorques(double *o_torques);

    /**
       \brief read gyro sensor output
       \param i_rank rank of gyro sensor
       \param o_rates array of angular velocities(length = 3) [rad/s]
    */
    void readGyroSensor(unsigned int i_rank, double *o_rates);

    /**
       \brief read accelerometer output
       \param i_rank rank of accelerometer
       \param o_accs array of accelerations(length = 3)[rad/s^2]
    */
    void readAccelerometer(unsigned int i_rank, double *o_accs);

    /**
       \brief read force sensor output
       \param i_rank rank of force sensor
       \param o_forces array of force/torque(length = 6)[N, Nm]
    */
    void readForceSensor(unsigned int i_rank, double *o_forces);

    /**
       \brief write array of reference angles of joint servo
       \param i_commands array of reference angles of joint servo[rad]
     */
    void writeJointCommands(const double *i_commands);

    /**
       \brief read array of reference angles of joint servo
       \param o_commands array of reference angles of joint servo[rad]
     */
    void readJointCommands(double *o_commands);

    /**
       \brief get length of extra servo states
       \param id joint id
       \return length of extra servo states
    */
    size_t lengthOfExtraServoState(int id);

    /**
       \brief read extra servo states
       \param id joint id
       \param state array of long where extra servo states are stored
    */
    void readExtraServoState(int id, long *state);

    /**
       \brief reasons of emergency
     */
    typedef enum {EMG_SERVO_ERROR, EMG_FZ} emg_reason;

    /**
       \brief check occurrence of emergency state
       \param o_reason kind of emergency source
       \param o_id id of sensor/joint of emergency source
       \return true if the robot is in emergency state, false otherwise
     */
    bool checkEmergency(emg_reason &o_reason, int &o_id);

    /**
       \brief set servo error limit value for specific joint or joint group
       \param i_jname joint name or joint group name
       \param i_limit new limit value[rad]
       \return true if set successfully, false otherwise 
     */
    bool setServoErrorLimit(const char *i_jname, double i_limit);

    void setProperty(const char *key, const char *value);
    bool addJointGroup(const char *gname, const std::vector<std::string>& jnames);
    std::vector<double> m_servoErrorLimit;  
    double m_fzLimitRatio;
    double m_maxZmpError;
private:
    /**
       \brief calibrate inertia sensor for one sampling period
     */
    void calibrateInertiaSensorOneStep();

    /**
       \brief check if a calibration process is running or not
       \true if one of calibration processes is running, false otherwise
     */
    bool isBusy() const;

    bool names2ids(const std::vector<std::string> &i_names, 
                   std::vector<int> &o_ids);

    void gain_control();
    void gain_control(int id);

    int calib_counter;
    std::vector<double> gain_counter;

    std::vector< boost::array<double,3> > gyro_sum;
    std::vector< boost::array<double,3> > accel_sum;
    std::vector< boost::array<double,3> > att_sum;

    std::vector<double> pgain;
    std::vector<double> dgain;

    int m_lLegForceSensorId, m_rLegForceSensorId;
    std::map<std::string, std::vector<int> > m_jointGroups;
    bool m_calibRequested;
    std::string m_calibJointName, m_calibOptions;

    boost::interprocess::interprocess_semaphore wait_sem;
};

#endif
