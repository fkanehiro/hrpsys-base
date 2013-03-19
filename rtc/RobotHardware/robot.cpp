#include <iostream>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <sys/time.h>
#include <hrpModel/Sensor.h>
#include <hrpModel/Link.h>
#include "defs.h"
#include "io/iob.h"
#include "robot.h"

#define CALIB_COUNT	(10*200)
#define GAIN_COUNT	( 5*200)

#define DEFAULT_MAX_ZMP_ERROR 0.03 //[m]
#define DEFAULT_ANGLE_ERROR_LIMIT 0.2 // [rad]

using namespace hrp;

robot::robot() : m_fzLimitRatio(0), m_maxZmpError(DEFAULT_MAX_ZMP_ERROR), m_calibRequested(false), m_pdgainsFilename("PDgains.sav"), wait_sem(0)
{
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
    for (i=0; i<numJoints(); i++){
	gain_counter[i] = GAIN_COUNT;
    }

    calib_counter = -1;

    pgain.resize(numJoints());
    dgain.resize(numJoints());
    old_pgain.resize(numJoints());
    old_dgain.resize(numJoints());
    default_pgain.resize(numJoints());
    default_dgain.resize(numJoints());
    for (int i=0; i<numJoints(); i++){
        pgain[i] = dgain[i] = 0.0;
        old_pgain[i] = old_dgain[i] = 0.0;
        default_pgain[i] = default_dgain[i] = 0.0;
    } 
    loadGain();
    for (int i=0; i<numJoints(); i++){
        pgain[i] = default_pgain[i];
        dgain[i] = default_dgain[i];
    }

    m_servoErrorLimit.resize(numJoints());
    for (int i=0; i<numJoints(); i++){
        m_servoErrorLimit[i] = DEFAULT_ANGLE_ERROR_LIMIT;
    }



    if (open_iob() == FALSE) return false;

    set_number_of_joints(numJoints());
    set_number_of_force_sensors(numSensors(Sensor::FORCE));
    set_number_of_gyro_sensors(numSensors(Sensor::RATE_GYRO));
    set_number_of_accelerometers(numSensors(Sensor::ACCELERATION));

    gyro_sum.resize(numSensors(Sensor::RATE_GYRO));
    accel_sum.resize(numSensors(Sensor::ACCELERATION));

    if ((number_of_joints() != numJoints())
	|| (number_of_force_sensors() != numSensors(Sensor::FORCE))
	|| (number_of_gyro_sensors() != numSensors(Sensor::RATE_GYRO))
	|| (number_of_accelerometers() != numSensors(Sensor::ACCELERATION))){
      std::cerr << "VRML and IOB are inconsistent" << std::endl;
      std::cerr << "  joints:" << numJoints() << "(VRML), " << number_of_joints() << "(IOB)"  << std::endl;
      std::cerr << "  force sensor:" << numSensors(Sensor::FORCE) << "(VRML), " << number_of_force_sensors() << "(IOB)"  << std::endl;
      std::cerr << "  gyro sensor:" << numSensors(Sensor::RATE_GYRO) << "(VRML), " << number_of_gyro_sensors() << "(IOB)"  << std::endl;
      std::cerr << "  accelerometer:" << numSensors(Sensor::ACCELERATION) << "(VRML), " << number_of_accelerometers() << "(IOB)"  << std::endl;
      return false;
    }

    return true;
}

void robot::removeForceSensorOffset()
{
    double force[6], offsets[6];
    
    for (int i=0; i<numSensors(Sensor::FORCE); i++) {
        read_force_sensor(i, force);
        for (int j=0; j<6; j++) offsets[j] = -force[j];
        write_force_offset(i, offsets);
    }
}

bool robot::loadGain()
{
    std::ifstream strm(m_pdgainsFilename.c_str());
    if (!strm.is_open()) {
        std::cerr << m_pdgainsFilename << " not found" << std::endl;
        return false;
    }

    double dummy;
    for (int i=0; i<numJoints(); i++){
        strm >> default_pgain[i];
        strm >> dummy;
        strm >> default_dgain[i];
    }
    strm.close();
    return true;
}

void robot::startInertiaSensorCalibration()
{
    if (numSensors(Sensor::ACCELERATION)==0 
        && numSensors(Sensor::RATE_GYRO)==0)  return;

    if (isBusy()) return;

    for (int j=0; j<numSensors(Sensor::RATE_GYRO); j++) {
        for (int i=0; i<3; i++) {
            gyro_sum[j][i] = 0;
        }
        write_gyro_sensor_offset(j, gyro_sum[j].data());
    }

    for(int j=0; j<numSensors(Sensor::ACCELERATION); j++) {
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

    calib_counter=CALIB_COUNT;

    wait_sem.wait();
}

void robot::initializeJointAngle(const char *name, const char *option)
{
    m_calibJointName = name;
    m_calibOptions   = option;
    m_calibRequested = true;
    wait_sem.wait();
}

void robot::calibrateInertiaSensorOneStep()
{
    if (calib_counter>0) {
        for (int j=0; j<numSensors(Sensor::RATE_GYRO); j++){
            double rate[3];
            read_gyro_sensor(j, rate);
            for (int i=0; i<3; i++)
                gyro_sum[j][i] += rate[i];
        }
        
        for (int j=0; j<numSensors(Sensor::ACCELERATION); j++){
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

        calib_counter--;
        if (calib_counter==0) {

            for (int j=0; j<numSensors(Sensor::RATE_GYRO); j++) {
                for (int i=0; i<3; i++) {
                    gyro_sum[j][i] = -gyro_sum[j][i]/CALIB_COUNT;
                }
                write_gyro_sensor_offset(j,  gyro_sum[j].data());
            }

            for (int j=0; j<numSensors(Sensor::ACCELERATION); j++) {
                for (int i=0; i<3; i++) {
                    accel_sum[j][i] = -accel_sum[j][i]/CALIB_COUNT;
#define	G	(9.8) // [m/s^2]
                    if (i==2) accel_sum[j][i] += G;
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

            wait_sem.post();
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
    for (i=0; i<numJoints(); i++) gain_control(i);
}

void robot::oneStep()
{
    calibrateInertiaSensorOneStep();
    gain_control();
    if (m_calibRequested){
        ::initializeJointAngle(m_calibJointName.c_str(), 
                               m_calibOptions.c_str());
        m_calibRequested = false;
        wait_sem.post();
    }
}

bool robot::servo(const char *jname, bool turnon)
{
    Link *l = NULL;
    if (strcmp(jname, "all") == 0 || strcmp(jname, "ALL") == 0){
        bool ret = true;
        for (int i=0; i<numJoints(); i++){
            ret = ret && servo(i, turnon);
        }
        return ret;
    }else if ((l = link(jname))){
        return servo(l->jointId, turnon);
    }else{
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
    if (jid == JID_INVALID || jid >= numJoints()) return false;
    
    int com = OFF;

    if (turnon) {
        com = ON;
    }

    write_pgain(jid, 0);
    write_dgain(jid, 0);
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
    if (jid == JID_INVALID || jid >= numJoints()) return false;
  
    int com = OFF;

    if (turnon) {
        for(int i=0; i<numJoints(); i++) {
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
            for (int i=0; i<numJoints(); i++) {
                write_pgain(i, 0);
                write_dgain(i, 0);
                write_servo(i, com);
                write_power_command(i, com);
            }
        } else       
            for (int i=0; i<numJoints(); i++)
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
    if (calib_counter > 0)
        return true;

    for (int i=0; i<numJoints(); i++) {
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

int robot::readJointTorques(double *o_torques)
{
    return read_actual_torques(o_torques);
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
    write_command_angles(i_commands);
}

void robot::readJointCommands(double *o_commands)
{
    read_command_angles(o_commands);
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

bool robot::checkEmergency(emg_reason &o_reason, int &o_id)
{
    int state;
    for (int i=0; i<numJoints(); i++){
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
        if (force[FZ] > totalMass()*G*m_fzLimitRatio){
	    std::cerr << time_string() << ": right Fz limit over: Fz = " << force[FZ] << std::endl;
            o_reason = EMG_FZ;
            o_id = m_rLegForceSensorId;
            return true;
        }
    } 
    if (m_lLegForceSensorId >= 0){
        double force[6];
        read_force_sensor(m_lLegForceSensorId, force);
        if (force[FZ] > totalMass()*G*m_fzLimitRatio){
	    std::cerr << time_string() << ": left Fz limit over: Fz = " << force[FZ] << std::endl;
            o_reason = EMG_FZ;
            o_id = m_lLegForceSensorId;
            return true;
        }
    } 
    return false;
}

bool robot::setServoGainPercentage(const char *i_jname, double i_percentage)
{
    if ( i_percentage < 0 && 100 < i_percentage ) {
        return false;
    }
    Link *l = NULL;
    if (strcmp(i_jname, "all") == 0 || strcmp(i_jname, "ALL") == 0){
        for (int i=0; i<numJoints(); i++){
            old_pgain[i] = pgain[i]; pgain[i] = default_pgain[i] * i_percentage/100.0;
            old_dgain[i] = dgain[i]; dgain[i] = default_dgain[i] * i_percentage/100.0;
            gain_counter[i] = 0;
        }
    }else if ((l = link(i_jname))){
        old_pgain[l->jointId] = pgain[l->jointId]; pgain[l->jointId] = default_pgain[l->jointId] * i_percentage/100.0;
        old_dgain[l->jointId] = dgain[l->jointId]; dgain[l->jointId] = default_dgain[l->jointId] * i_percentage/100.0;
        gain_counter[l->jointId] = 0;
    }else{
        const std::vector<int> jgroup = m_jointGroups[i_jname];
        if (jgroup.size()==0) return false;
        for (unsigned int i=0; i<jgroup.size(); i++){
            old_pgain[jgroup[i]] = pgain[jgroup[i]]; pgain[jgroup[i]] = default_pgain[jgroup[i]] * i_percentage/100.0;
            old_dgain[jgroup[i]] = dgain[jgroup[i]]; dgain[jgroup[i]] = default_dgain[jgroup[i]] * i_percentage/100.0;
            gain_counter[jgroup[i]] = 0;
        }
    }
    return true;
}

bool robot::setServoErrorLimit(const char *i_jname, double i_limit)
{
    Link *l = NULL;
    if (strcmp(i_jname, "all") == 0 || strcmp(i_jname, "ALL") == 0){
        for (int i=0; i<numJoints(); i++){
            m_servoErrorLimit[i] = i_limit;
        }
    }else if ((l = link(i_jname))){
        m_servoErrorLimit[l->jointId] = i_limit;
    }else{
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

