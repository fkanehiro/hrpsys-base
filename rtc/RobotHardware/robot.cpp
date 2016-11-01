#include <iostream>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <sys/time.h>
#include <hrpModel/Sensor.h>
#include <hrpModel/Link.h>
#include "defs.h"
#include "hrpsys/io/iob.h"
#include "robot.h"
#include "hrpsys/util/Hrpsys.h"

#define CALIB_COUNT	(10*200)
#define GAIN_COUNT	( 5*200)

#define DEFAULT_MAX_ZMP_ERROR 0.03 //[m]
#define DEFAULT_ANGLE_ERROR_LIMIT 0.2 // [rad]

using namespace hrp;


robot::robot(double dt) : m_fzLimitRatio(0), m_maxZmpError(DEFAULT_MAX_ZMP_ERROR), m_accLimit(0), m_calibRequested(false), m_pdgainsFilename("PDgains.sav"), m_reportedEmergency(true), m_dt(dt), m_enable_poweroff_check(false)
{
    sem_init(&wait_sem, 0, 0);
    m_rLegForceSensorId = m_lLegForceSensorId = -1;
}

robot::~robot()
{
    close_iob();
}

bool robot::init()
{
    int i;
    gain_counter.resize(numJoints());
    for (unsigned i=0; i<numJoints(); i++){
	gain_counter[i] = GAIN_COUNT;
    }

    inertia_calib_counter = force_calib_counter = -1;

    pgain.resize(numJoints());
    dgain.resize(numJoints());
    old_pgain.resize(numJoints());
    old_dgain.resize(numJoints());
    default_pgain.resize(numJoints());
    default_dgain.resize(numJoints());
    for (unsigned int i=0; i<numJoints(); i++){
        pgain[i] = dgain[i] = 0.0;
        old_pgain[i] = old_dgain[i] = 0.0;
        default_pgain[i] = default_dgain[i] = 0.0;
    } 
    loadGain();
    for (unsigned int i=0; i<numJoints(); i++){
        pgain[i] = default_pgain[i];
        dgain[i] = default_dgain[i];
    }

    m_servoErrorLimit.resize(numJoints());
    for (unsigned int i=0; i<numJoints(); i++){
        m_servoErrorLimit[i] = DEFAULT_ANGLE_ERROR_LIMIT;
    }



    set_number_of_joints(numJoints());
    set_number_of_force_sensors(numSensors(Sensor::FORCE));
    set_number_of_gyro_sensors(numSensors(Sensor::RATE_GYRO));
    set_number_of_accelerometers(numSensors(Sensor::ACCELERATION));

    gyro_sum.resize(numSensors(Sensor::RATE_GYRO));
    accel_sum.resize(numSensors(Sensor::ACCELERATION));
    force_sum.resize(numSensors(Sensor::FORCE));

    if ((number_of_joints() != (int)numJoints())
	|| (number_of_force_sensors() != (int)numSensors(Sensor::FORCE))
	|| (number_of_gyro_sensors() != (int)numSensors(Sensor::RATE_GYRO))
	|| (number_of_accelerometers() != (int)numSensors(Sensor::ACCELERATION))){
      std::cerr << "VRML and IOB are inconsistent" << std::endl;
      std::cerr << "  joints:" << numJoints() << "(VRML), " << number_of_joints() << "(IOB)"  << std::endl;
      std::cerr << "  force sensor:" << numSensors(Sensor::FORCE) << "(VRML), " << number_of_force_sensors() << "(IOB)"  << std::endl;
      std::cerr << "  gyro sensor:" << numSensors(Sensor::RATE_GYRO) << "(VRML), " << number_of_gyro_sensors() << "(IOB)"  << std::endl;
      std::cerr << "  accelerometer:" << numSensors(Sensor::ACCELERATION) << "(VRML), " << number_of_accelerometers() << "(IOB)"  << std::endl;
      return false;
    }
    G << 0, 0, 9.8;

    if (open_iob() == FALSE) return false;

    return true;
}

void robot::removeForceSensorOffset()
{
    std::cerr << "[RobotHardware] removeForceSensorOffset..." << std::endl;
    startForceSensorCalibration();
    std::cerr << "[RobotHardware] removeForceSensorOffset...done." << std::endl;
}

bool robot::loadGain()
{
    std::ifstream strm(m_pdgainsFilename.c_str());
    if (!strm.is_open()) {
        std::cerr << m_pdgainsFilename << " not found" << std::endl;
        return false;
    }

    double dummy;
    for (unsigned int i=0; i<numJoints(); i++){
        strm >> default_pgain[i];
        strm >> dummy;
        strm >> default_dgain[i];
    }
    strm.close();
    // Print loaded gain
    std::cerr << "[RobotHardware] loadGain" << std::endl;
    for (unsigned int i=0; i<numJoints(); i++) {                                                                                                                                               std::cerr << "[RobotHardware]   " << joint(i)->name << ", pgain = " << default_pgain[i] << ", dgain = " << default_dgain[i] << std::endl;
    }
    return true;
}

void robot::startInertiaSensorCalibration()
{
    std::cerr << "[RobotHardware] startInertiaSensorCalibration..." << std::endl;
    if (numSensors(Sensor::ACCELERATION)==0 
        && numSensors(Sensor::RATE_GYRO)==0)  return;

    if (isBusy()) return;

    for (unsigned int j=0; j<numSensors(Sensor::RATE_GYRO); j++) {
        for (int i=0; i<3; i++) {
            gyro_sum[j][i] = 0;
        }
        write_gyro_sensor_offset(j, gyro_sum[j].data());
    }

    for(unsigned int j=0; j<numSensors(Sensor::ACCELERATION); j++) {
        for (int i=0; i<3; i++) {
            accel_sum[j][i] = 0;
        }
        write_accelerometer_offset(j, accel_sum[j].data());
    }

#if 0
    for(unsigned int j=0; j<m_natt; j++) {
        for (int i=0; i<3; i++) {
            att_sum[j][i] = 0;
        }
        write_attitude_sensor_offset(j, att_sum[j]);
    }
#endif

    inertia_calib_counter=CALIB_COUNT;

    sem_wait(&wait_sem);
    std::cerr << "[RobotHardware] startInertiaSensorCalibration...done." << std::endl;
}

void robot::startForceSensorCalibration()
{
    if (numSensors(Sensor::FORCE)==0)  return;

    if (isBusy()) return;

    for (unsigned int j=0; j<numSensors(Sensor::FORCE); j++) {
        for (int i=0; i<6; i++) {
            force_sum[j][i] = 0;
        }
    }

    force_calib_counter=CALIB_COUNT;

    sem_wait(&wait_sem);
}

void robot::initializeJointAngle(const char *name, const char *option)
{
    m_calibJointName = name;
    m_calibOptions   = option;
    m_calibRequested = true;
    sem_wait(&wait_sem);
}

void robot::calibrateInertiaSensorOneStep()
{
    if (inertia_calib_counter>0) {
        for (unsigned int j=0; j<numSensors(Sensor::RATE_GYRO); j++){
            double rate[3];
            read_gyro_sensor(j, rate);
            for (int i=0; i<3; i++)
                gyro_sum[j][i] += rate[i];
        }
        
        for (unsigned int j=0; j<numSensors(Sensor::ACCELERATION); j++){
            double acc[3];
            read_accelerometer(j, acc);
            for (int i=0; i<3; i++)
                accel_sum[j][i] += acc[i];
        }

#if 0
        for (unsigned int j=0; j<m_natt; j++){
            double att[3];
            read_attitude_sensor(j, att);
            for (int i=0; i<3; i++)
                att_sum[j][i] += att[i];
        }
#endif

        inertia_calib_counter--;
        if (inertia_calib_counter==0) {

            for (unsigned int j=0; j<numSensors(Sensor::RATE_GYRO); j++) {
                for (int i=0; i<3; i++) {
                    gyro_sum[j][i] = -gyro_sum[j][i]/CALIB_COUNT;
                }
                write_gyro_sensor_offset(j,  gyro_sum[j].data());
            }

            for (unsigned int j=0; j<numSensors(Sensor::ACCELERATION); j++) {
                hrp::Sensor* m_sensor = sensor(hrp::Sensor::ACCELERATION, j);
                hrp::Matrix33 m_sensorR = m_sensor->link->R * m_sensor->localR;
                hrp::Vector3 G_rotated = m_sensorR.transpose() * G;
                for (int i=0; i<3; i++) {
                    accel_sum[j][i] = -accel_sum[j][i]/CALIB_COUNT + G_rotated(i);
                }
                write_accelerometer_offset(j, accel_sum[j].data());
            }

#if 0
            for (unsigned int j=0; j<m_natt; j++) {
                for (int i=0; i<3; i++) {
                    att_sum[j][i] = -att_sum[j][i]/CALIB_COUNT;
                }
                write_attitude_sensor_offset(j, att_sum[j]);
            }
#endif

            sem_post(&wait_sem);
        }
    }
}

void robot::calibrateForceSensorOneStep()
{
    if (force_calib_counter>0) {
        for (unsigned int j=0; j<numSensors(Sensor::FORCE); j++){
            double force[6];
            read_force_sensor(j, force);
            for (int i=0; i<6; i++)
                force_sum[j][i] += force[i];
        }
        force_calib_counter--;
        if (force_calib_counter==0) {

            for (unsigned int j=0; j<numSensors(Sensor::FORCE); j++) {
                for (int i=0; i<6; i++) {
                    force_sum[j][i] = -force_sum[j][i]/CALIB_COUNT;
                }
                write_force_offset(j,  force_sum[j].data());
            }

            sem_post(&wait_sem);
        }
    }
}

void robot::gain_control(int i)
{
    double new_pgain=0,new_dgain=0;
    if (gain_counter[i] < GAIN_COUNT){
        gain_counter[i]++;
        new_pgain = (pgain[i]-old_pgain[i])*gain_counter[i]/GAIN_COUNT + old_pgain[i];
        new_dgain = (dgain[i]-old_dgain[i])*gain_counter[i]/GAIN_COUNT + old_dgain[i];
        write_pgain(i, new_pgain);
        write_dgain(i, new_dgain);
    }
}

void robot::gain_control()
{
    int i;
    for (unsigned i=0; i<numJoints(); i++) gain_control(i);
}

void robot::oneStep()
{
    calibrateInertiaSensorOneStep();
    calibrateForceSensorOneStep();
    gain_control();
    if (m_calibRequested){
        ::initializeJointAngle(m_calibJointName.c_str(), 
                               m_calibOptions.c_str());
        m_calibRequested = false;
        sem_post(&wait_sem);
    }
}

bool robot::servo(const char *jname, bool turnon)
{
    Link *l = NULL;
    if (strcmp(jname, "all") == 0 || strcmp(jname, "ALL") == 0){
        bool ret = true;
        for (unsigned int i=0; i<numJoints(); i++){
            ret = ret && servo(i, turnon);
        }
        m_reportedEmergency = false;
        return ret;
    }else if ((l = link(jname))){
        return servo(l->jointId, turnon);
    }else{
        char *s = (char *)jname; while(*s) {*s=toupper(*s);s++;}
        const std::vector<int> jgroup = m_jointGroups[jname];
        if (jgroup.size() == 0) return false;
        bool ret = true;
        for (unsigned int i=0; i<jgroup.size(); i++){
            ret = ret && servo(jgroup[i], turnon);
            return ret;
        }
    }
    return false;
}

bool robot::servo(int jid, bool turnon)
{
    if (jid == JID_INVALID || jid >= (int)numJoints()) return false;
    
    int com = OFF;

    if (turnon) {
        com = ON;
    }

    write_pgain(jid, 0);
    write_dgain(jid, 0);
    old_pgain[jid] = 0;
    old_dgain[jid] = 0;
    if (turnon){
        double angle;
        read_actual_angle(jid, &angle);
            write_command_angle(jid, angle);
    }
    write_servo(jid, com);
    if (turnon) gain_counter[jid] = 0;

    return true;
}


bool robot::power(const char *jname, bool turnon)
{
    int jid = JID_INVALID;

    if (strcmp(jname, "all") == 0 || strcmp(jname, "ALL") == 0){
        jid = JID_ALL;
    }else{
        Link *l = link(jname);
        if (!l) return false;
        jid = l->jointId;
    }
    return power(jid, turnon);
}

bool robot::power(int jid, bool turnon)
{
    if (jid == JID_INVALID || jid >= (int)numJoints()) return false;
  
    int com = OFF;

    if (turnon) {
        for(unsigned int i=0; i<numJoints(); i++) {
            int s;
            read_servo_state(i, &s);
            if (s == ON)
                return false;
        }
        com = ON;
    }

    // next part written as if there is a relay for each joint
    // up to HRP2, TREX, PARASA there is only one.

    if(jid == JID_ALL) {
        if (com == OFF) {
            for (unsigned int i=0; i<numJoints(); i++) {
                write_pgain(i, 0);
                write_dgain(i, 0);
                write_servo(i, com);
                write_power_command(i, com);
            }
        } else       
            for (unsigned int i=0; i<numJoints(); i++)
                write_power_command(i, com);
    } else {
        if (com == OFF) {
            write_pgain(jid, 0);
            write_dgain(jid, 0);
            write_servo(jid, com);
            write_power_command(jid, com);
        } else {
            write_power_command(jid, com);
        }
    }

    return true;
}

bool robot::isBusy() const
{
    if (inertia_calib_counter > 0 || force_calib_counter > 0)
        return true;

    for (unsigned int i=0; i<numJoints(); i++) {
        if (gain_counter[i] < GAIN_COUNT) {
            return true;
        }
    }

    return false; 
}

void robot::readJointAngles(double *o_angles)
{
    read_actual_angles(o_angles);
}

void robot::readJointVelocities(double *o_velocities)
{
    read_actual_velocities(o_velocities);
}

int robot::readJointTorques(double *o_torques)
{
    return read_actual_torques(o_torques);
}

int robot::readJointCommandTorques(double *o_torques)
{
    return read_command_torques(o_torques);
}

void robot::readGyroSensor(unsigned int i_rank, double *o_rates)
{
    read_gyro_sensor(i_rank, o_rates);
}

void robot::readAccelerometer(unsigned int i_rank, double *o_accs)
{
    read_accelerometer(i_rank, o_accs);
}

void robot::readForceSensor(unsigned int i_rank, double *o_forces)
{
    read_force_sensor(i_rank, o_forces);
}

void robot::writeJointCommands(const double *i_commands)
{
    if (!m_commandOld.size()) {
        m_commandOld.resize(numJoints());
        m_velocityOld.resize(numJoints());
    }
    for (unsigned int i=0; i<numJoints(); i++){
        m_velocityOld[i] = (i_commands[i] - m_commandOld[i])/m_dt;
        m_commandOld[i] = i_commands[i];
    }
    write_command_angles(i_commands);
}

void robot::readJointCommands(double *o_commands)
{
    read_command_angles(o_commands);
}

void robot::writeTorqueCommands(const double *i_commands)
{
    write_command_torques(i_commands);
}

void robot::writeVelocityCommands(const double *i_commands)
{
    write_command_velocities(i_commands);
}

void robot::readPowerStatus(double &o_voltage, double &o_current)
{
    read_power(&o_voltage, &o_current);
}

int robot::readCalibState(int i)
{
    int v=0;
    read_calib_state(i, &v);
    return v;
}

int robot::readPowerState(int i)
{
    int v=0;
    read_power_state(i, &v);
    return v;
}

int robot::readServoState(int i)
{
    int v=0;
    read_servo_state(i, &v);
    return v;
}

int robot::readServoAlarm(int i)
{
    int v=0;
    read_servo_alarm(i, &v);
    return v;
}

int robot::readDriverTemperature(int i)
{
    unsigned char temp=0;
    read_driver_temperature(i, &temp);
    return temp;
}

char *time_string()
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    struct tm *tm_ = localtime(&tv.tv_sec);
    static char time[20];
    sprintf(time, "%02d:%02d:%02d.%06d", tm_->tm_hour, tm_->tm_min, tm_->tm_sec, (int)tv.tv_usec);
    return time;
}

bool robot::checkJointCommands(const double *i_commands)
{
    if (!m_dt) return false;
    if (!m_commandOld.size()) return false;

    int state;
    for (unsigned int i=0; i<numJoints(); i++){
        read_servo_state(i, &state);
        if (state == ON){
            double command_old=m_commandOld[i], command=i_commands[i];
            double v = (command - command_old)/m_dt;
            if (fabs(v) > joint(i)->uvlimit){
                std::cerr << time_string()
                          << ": joint command velocity limit over: joint = " 
		          << joint(i)->name
		          << ", vlimit = " << joint(i)->uvlimit/M_PI*180 
                          << "[deg/s], v = " 
		          << v/M_PI*180 << "[deg/s]" << std::endl;
                return true;
            }
            double a = (v - m_velocityOld[i])/m_dt;
            if (m_accLimit && fabs(a) > m_accLimit){
                std::cerr << time_string()
                          << ": joint command acceleration limit over: joint = " 
		          << joint(i)->name
		          << ", alimit = " << m_accLimit/M_PI*180 
                          << "[deg/s^2], v = " 
		          << a/M_PI*180 << "[deg/s^2]" << std::endl;
                return true;
            }
        }
    }
    return false;
}

bool robot::checkEmergency(emg_reason &o_reason, int &o_id)
{
    int state;
    for (unsigned int i=0; i<numJoints(); i++){
        read_servo_state(i, &state);
        if (state == ON && m_servoErrorLimit[i] != 0){
            double angle, command;
            read_actual_angle(i, &angle);
            read_command_angle(i, &command);
            if (fabs(angle-command) > m_servoErrorLimit[i]){
                std::cerr << time_string()
                          << ": servo error limit over: joint = " 
		          << joint(i)->name
		          << ", qRef = " << command/M_PI*180 << "[deg], q = " 
		          << angle/M_PI*180 << "[deg]" << std::endl;
                o_reason = EMG_SERVO_ERROR;
                o_id = i;
                return true;
            }
        }
    }

    if (m_rLegForceSensorId >= 0){
        double force[6];
        read_force_sensor(m_rLegForceSensorId, force);
        if (force[FZ] > totalMass()*G(2)*m_fzLimitRatio){
	    std::cerr << time_string() << ": right Fz limit over: Fz = " << force[FZ] << std::endl;
            o_reason = EMG_FZ;
            o_id = m_rLegForceSensorId;
            return true;
        }
    } 
    if (m_lLegForceSensorId >= 0){
        double force[6];
        read_force_sensor(m_lLegForceSensorId, force);
        if (force[FZ] > totalMass()*G(2)*m_fzLimitRatio){
	    std::cerr << time_string() << ": left Fz limit over: Fz = " << force[FZ] << std::endl;
            o_reason = EMG_FZ;
            o_id = m_lLegForceSensorId;
            return true;
        }
    } 
    int alarm;
    for (unsigned int i=0; i<numJoints(); i++){
        if (!read_servo_alarm(i, &alarm)) continue;
        if (alarm & SS_EMERGENCY) {
            if (!m_reportedEmergency) {
                m_reportedEmergency = true;
                o_reason = EMG_SERVO_ALARM;
                o_id = i;
            }
            return true;
        }
    }
    m_reportedEmergency = false;
    // Power state check
    if (m_enable_poweroff_check) {
      int pstate, sstate;
      for (unsigned int i=0; i<numJoints(); i++){
        read_power_state(i, &pstate);
        read_servo_state(i, &sstate);
        // If power OFF while servo ON
        if (!m_reportedEmergency && (pstate == OFF) && (sstate == ON) ) {
          m_reportedEmergency = true;
          o_reason = EMG_POWER_OFF;
          o_id = i;
          std::cerr << time_string() << ": power off detected : joint = " << joint(i)->name << std::endl;
          return true;
        }
      }
      m_reportedEmergency = false;
    }
    return false;
}

bool robot::setServoGainPercentage(const char *i_jname, double i_percentage)
{
    if ( i_percentage < 0 && 100 < i_percentage ) {
        std::cerr << "[RobotHardware] Invalid percentage " <<  i_percentage << "[%] for setServoGainPercentage. Percentage should be in (0, 100)[%]." << std::endl;
        return false;
    }
    Link *l = NULL;
    if (strcmp(i_jname, "all") == 0 || strcmp(i_jname, "ALL") == 0){
        for (unsigned int i=0; i<numJoints(); i++){
            if (!read_pgain(i, &old_pgain[i])) old_pgain[i] = pgain[i];
            pgain[i] = default_pgain[i] * i_percentage/100.0;
            if (!read_dgain(i, &old_dgain[i])) old_dgain[i] = dgain[i];
            dgain[i] = default_dgain[i] * i_percentage/100.0;
            gain_counter[i] = 0;
        }
        std::cerr << "[RobotHardware] setServoGainPercentage " << i_percentage << "[%] for all joints" << std::endl;
    }else if ((l = link(i_jname))){
        if (!read_pgain(l->jointId, &old_pgain[l->jointId])) old_pgain[l->jointId] = pgain[l->jointId];
        pgain[l->jointId] = default_pgain[l->jointId] * i_percentage/100.0;
        if (!read_dgain(l->jointId, &old_dgain[l->jointId])) old_dgain[l->jointId] = dgain[l->jointId];
        dgain[l->jointId] = default_dgain[l->jointId] * i_percentage/100.0;
        gain_counter[l->jointId] = 0;
        std::cerr << "[RobotHardware] setServoGainPercentage " << i_percentage << "[%] for " << i_jname << std::endl;
    }else{
        char *s = (char *)i_jname; while(*s) {*s=toupper(*s);s++;}
        const std::vector<int> jgroup = m_jointGroups[i_jname];
        if (jgroup.size()==0) return false;
        for (unsigned int i=0; i<jgroup.size(); i++){
            if (!read_pgain(jgroup[i], &old_pgain[jgroup[i]])) old_pgain[jgroup[i]] = pgain[jgroup[i]];
            pgain[jgroup[i]] = default_pgain[jgroup[i]] * i_percentage/100.0;
            if (!read_dgain(jgroup[i], &old_dgain[jgroup[i]])) old_dgain[jgroup[i]] = dgain[jgroup[i]];
            dgain[jgroup[i]] = default_dgain[jgroup[i]] * i_percentage/100.0;
            gain_counter[jgroup[i]] = 0;
        }
        std::cerr << "[RobotHardware] setServoGainPercentage " << i_percentage << "[%] for " << i_jname << std::endl;
    }
    return true;
}

bool robot::setServoErrorLimit(const char *i_jname, double i_limit)
{
    Link *l = NULL;
    if (strcmp(i_jname, "all") == 0 || strcmp(i_jname, "ALL") == 0){
        for (unsigned int i=0; i<numJoints(); i++){
            m_servoErrorLimit[i] = i_limit;
        }
    }else if ((l = link(i_jname))){
        m_servoErrorLimit[l->jointId] = i_limit;
    }else{
        char *s = (char *)i_jname; while(*s) {*s=toupper(*s);s++;}
        const std::vector<int> jgroup = m_jointGroups[i_jname];
        if (jgroup.size()==0) return false;
        for (unsigned int i=0; i<jgroup.size(); i++){
            m_servoErrorLimit[jgroup[i]] = i_limit;
        }
    }
    return true;
}

void robot::setProperty(const char *i_key, const char *i_value)
{
    std::istringstream iss(i_value);
    std::string key(i_key);
    bool isKnownKey = true;
    if (key == "sensor_id.right_leg_force_sensor"){
        iss >> m_rLegForceSensorId;
    }else if (key == "sensor_id.left_leg_force_sensor"){
        iss >> m_lLegForceSensorId;
    }else if (key == "pdgains.file_name"){
        iss >> m_pdgainsFilename;
    }else if (key == "enable_poweroff_check"){
        std::string tmp;
        iss >> tmp;
        m_enable_poweroff_check = (tmp=="true");
    }else{
        isKnownKey = false;
    }
    if (isKnownKey) std::cout << i_key << ": " << i_value << std::endl;
}

bool robot::names2ids(const std::vector<std::string> &i_names, 
                            std::vector<int> &o_ids)
{ 
    bool ret = true;
    for (unsigned int i=0; i<i_names.size(); i++){
        Link *l = link(i_names[i].c_str());
        if (!l){
            std::cout << "joint named [" << i_names[i] << "] not found" 
                      << std::endl;
            ret = false;
        }else{
            o_ids.push_back(l->jointId);
        }
    }
    return ret;
}

bool robot::addJointGroup(const char *gname, const std::vector<std::string>& jnames)
{
    char *s = (char *)gname; while(*s) {*s=toupper(*s);s++;}
    std::vector<int> jids;
    bool ret = names2ids(jnames, jids);
    m_jointGroups[gname] = jids;
    return ret;
}

size_t robot::lengthOfExtraServoState(int id)
{
    return length_of_extra_servo_state(id);
}

void robot::readExtraServoState(int id, int *state)
{
    read_extra_servo_state(id, state);
}

bool robot::readDigitalInput(char *o_din)
{
#ifndef NO_DIGITAL_INPUT
    return read_digital_input(o_din);
#else
    return false;
#endif
}

int robot::lengthDigitalInput()
{
#ifndef NO_DIGITAL_INPUT
    return length_digital_input();
#else
    return 0;
#endif
}

bool robot::writeDigitalOutput(const char *i_dout)
{
    return write_digital_output(i_dout);
}

bool robot::writeDigitalOutputWithMask(const char *i_dout, const char *i_mask)
{
    return write_digital_output_with_mask(i_dout, i_mask);
}

int robot::lengthDigitalOutput()
{
    return length_digital_output();
}

bool robot::readDigitalOutput(char *o_dout)
{
    return read_digital_output(o_dout);
}

void robot::readBatteryState(unsigned int i_rank, double &voltage, 
                             double &current, double &soc)
{
#if defined(ROBOT_IOB_VERSION) && ROBOT_IOB_VERSION >= 2
    read_battery(i_rank, &voltage, &current, &soc);
#else
    voltage=0; current=0; soc=0;
#endif
}

int robot::numBatteries()
{
#if defined(ROBOT_IOB_VERSION) && ROBOT_IOB_VERSION >= 2
    return number_of_batteries();
#else
    return 0;
#endif
}

void robot::readThermometer(unsigned int i_rank, double &o_temp)
{
#if defined(ROBOT_IOB_VERSION) && ROBOT_IOB_VERSION >= 2
    read_temperature(i_rank, &o_temp);
#else
    o_temp=0;
#endif
}

int robot::numThermometers()
{
#if defined(ROBOT_IOB_VERSION) && ROBOT_IOB_VERSION >= 2
    return number_of_thermometers();
#else
    return 0;
#endif
}
