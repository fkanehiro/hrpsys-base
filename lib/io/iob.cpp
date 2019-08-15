#include <unistd.h>
#include <iostream>
#include <cstdlib>
#include <cstring>
#include <vector>
#include "iob.h"

static std::vector<double> command;
static std::vector<std::vector<double> > forces;
static std::vector<std::vector<double> > gyros;
static std::vector<std::vector<double> > accelerometers;
static std::vector<std::vector<double> > attitude_sensors;
static std::vector<std::vector<double> > force_offset;
static std::vector<std::vector<double> > gyro_offset;
static std::vector<std::vector<double> > accel_offset;
static std::vector<int> power;
static std::vector<int> servo;
static bool isLocked = false;
static int frame = 0;
static timespec g_ts;
static long g_period_ns=5000000;

#define CHECK_JOINT_ID(id) if ((id) < 0 || (id) >= number_of_joints()) return E_ID
#define CHECK_FORCE_SENSOR_ID(id) if ((id) < 0 || (id) >= number_of_force_sensors()) return E_ID
#define CHECK_ACCELEROMETER_ID(id) if ((id) < 0 || (id) >= number_of_accelerometers()) return E_ID
#define CHECK_GYRO_SENSOR_ID(id) if ((id) < 0 || (id) >= number_of_gyro_sensors()) return E_ID
#define CHECK_ATTITUDE_SENSOR_ID(id) if ((id) < 0 || (id) >= number_of_attitude_sensors()) return E_ID

#if (defined __APPLE__)
#include <mach/mach_time.h>  
int clock_gettime(clockid_t clk_id, struct timespec *tp)
{
    if (clk_id != CLOCK_MONOTONIC) return -1;

    uint64_t clk;
    clk = mach_absolute_time();  

    static mach_timebase_info_data_t info = {0,0};  
    if (info.denom == 0) mach_timebase_info(&info);  
  
    uint64_t elapsednano = clk * (info.numer / info.denom);  
  
    tp->tv_sec = elapsednano * 1e-9;  
    tp->tv_nsec = elapsednano - (tp->tv_sec * 1e9);  
    return 0;
}

#define TIMER_ABSTIME 0
int clock_nanosleep(clockid_t clk_id, int flags, struct timespec *tp,
    struct timespec *remain)
{
    if (clk_id != CLOCK_MONOTONIC || flags != TIMER_ABSTIME) return -1;

    static mach_timebase_info_data_t info = {0,0};  
    if (info.denom == 0) mach_timebase_info(&info);  
  
    uint64_t clk = (tp->tv_sec*1e9 + tp->tv_nsec)/(info.numer/info.denom);
    
    mach_wait_until(clk);
    return 0;
}
#endif


int number_of_joints()
{
    return (int)command.size();
}

int number_of_force_sensors()
{
    return (int)forces.size();
}

int number_of_gyro_sensors()
{
    return (int)gyros.size();
}

int number_of_accelerometers()
{
    return (int)accelerometers.size();
}

int number_of_attitude_sensors()
{
    return (int)attitude_sensors.size();
}

int set_number_of_joints(int num)
{
    command.resize(num);
    power.resize(num);
    servo.resize(num);
    for (int i=0; i<num; i++){
        command[i] = power[i] = servo[i] = 0;
    }
    return TRUE;
}

int set_number_of_force_sensors(int num)
{
    forces.resize(num);
    force_offset.resize(num);
    for (unsigned int i=0; i<forces.size();i++){
        forces[i].resize(6);
        force_offset[i].resize(6);
        for (int j=0; j<6; j++){
            forces[i][j] = 0;
            force_offset[i][j] = 0;
        }
    }
    return TRUE;
}

int set_number_of_gyro_sensors(int num)
{
    gyros.resize(num);
    gyro_offset.resize(num);
    for (unsigned int i=0; i<gyros.size();i++){
        gyros[i].resize(3);
        gyro_offset[i].resize(3);
        for (int j=0; j<3; j++){
            gyros[i][j] = 0.0;
            gyro_offset[i][j] = 0.0;
        }
    }
    return TRUE;
}

int set_number_of_accelerometers(int num)
{
    accelerometers.resize(num);
    accel_offset.resize(num);
    for (unsigned int i=0; i<accelerometers.size();i++){
        accelerometers[i].resize(3);
        accel_offset[i].resize(3);
        for (int j=0; j<3; j++){
            accelerometers[i][j] = j == 2 ? 9.81 : 0.0;
            accel_offset[i][j] = 0;
        }
    }
    return TRUE;
}

int set_number_of_attitude_sensors(int num)
{
    attitude_sensors.resize(num);
    for (unsigned int i=0; i<attitude_sensors.size();i++){
        attitude_sensors[i].resize(3);
        for (int j=0; j<3; j++){
            attitude_sensors[i][j] = 0.0;
        }
    }
    return TRUE;
}

int read_power_state(int id, int *s)
{
    CHECK_JOINT_ID(id);
    *s = power[id];
    return TRUE;
}

int write_power_command(int id, int com)
{
    CHECK_JOINT_ID(id);
    power[id] = com;
    return TRUE;
}

int read_power_command(int id, int *com)
{
    CHECK_JOINT_ID(id);
    *com = power[id];
    return TRUE;
}

int read_servo_state(int id, int *s)
{
    CHECK_JOINT_ID(id);
    *s = servo[id];
    return TRUE;
}

int read_servo_alarm(int id, int *a)
{
    CHECK_JOINT_ID(id);
    *a = 0;
    return TRUE;
}
    
int read_control_mode(int id, joint_control_mode *s)
{
    CHECK_JOINT_ID(id);
    *s = JCM_POSITION;
    return TRUE;
}

int write_control_mode(int id, joint_control_mode s)
{
    CHECK_JOINT_ID(id);
    return TRUE;
}

int read_actual_angle(int id, double *angle)
{
    CHECK_JOINT_ID(id);
    *angle = command[id]+0.01;
    return TRUE;
}

int read_actual_angles(double *angles)
{
    for (int i=0; i<number_of_joints(); i++){
        angles[i] = command[i]+0.01;
    }
    return TRUE;
}

int read_actual_torques(double *torques)
{
    return FALSE;
}

int read_command_torque(int id, double *torque)
{
    return FALSE;
}

int write_command_torque(int id, double torque)
{
    return FALSE;
}

int read_command_torques(double *torques)
{
    return FALSE;
}

int write_command_torques(const double *torques)
{
    return FALSE;
}

int read_command_angle(int id, double *angle)
{
    CHECK_JOINT_ID(id);
    *angle = command[id];
    return TRUE;
}

int write_command_angle(int id, double angle)
{
    CHECK_JOINT_ID(id);
    command[id] = angle;
    return TRUE;
}

int read_command_angles(double *angles)
{
    for (int i=0; i<number_of_joints(); i++){
        angles[i] = command[i];
    }
    return TRUE;
}

int write_command_angles(const double *angles)
{
    for (int i=0; i<number_of_joints(); i++){
        command[i] = angles[i];
    }
    return TRUE;
}

int read_pgain(int id, double *gain)
{
    return FALSE;
}

int write_pgain(int id, double gain)
{
    return FALSE;
}

int read_dgain(int id, double *gain)
{
    return FALSE;
}

int write_dgain(int id, double gain)
{
    return FALSE;
}

int read_force_sensor(int id, double *forces)
{
    for (int i=0; i<6; i++){
        forces[i] = ((double)random()-RAND_MAX/2)/(RAND_MAX/2)*2 
            + 2 + force_offset[id][i]; // 2 = initial offset
    }
    return TRUE;
}

int read_gyro_sensor(int id, double *rates)
{
    CHECK_GYRO_SENSOR_ID(id);
    for (int i=0; i<3; i++){
        rates[i] = ((double)random()-RAND_MAX/2)/(RAND_MAX/2)*0.01
            + 0.01 + gyro_offset[id][i]; // 0.01 = initial offset
    }
    return TRUE;
}

int read_accelerometer(int id, double *accels)
{
    for (int i=0; i<3; i++){
        double randv = ((double)random()-RAND_MAX/2)/(RAND_MAX/2)*0.01;
        accels[i] = (i == 2 ? (9.8+randv) : randv) 
            + 0.01 + accel_offset[id][i]; // 0.01 = initial offset
    }
    return TRUE;
}

int read_touch_sensors(unsigned short *onoff)
{
    return FALSE;
}

int read_attitude_sensor(int id, double *att)
{
    return FALSE;
}

int read_current(int id, double *mcurrent)
{
    return FALSE;
}

int read_current_limit(int id, double *v)
{
    return FALSE;
}

int read_currents(double *currents)
{
    return FALSE;
}

int read_gauges(double *gauges)
{
    return FALSE;
}

int read_actual_velocity(int id, double *vel)
{
    return FALSE;
}

int read_command_velocity(int id, double *vel)
{
    return FALSE;
}

int write_command_velocity(int id, double vel)
{
    return FALSE;
}

int read_actual_velocities(double *vels)
{
    return FALSE;
}

int read_command_velocities(double *vels)
{
    return FALSE;
}

int write_command_velocities(const double *vels)
{
    return FALSE;
}

int read_temperature(int id, double *v)
{
    return FALSE;
}

int write_servo(int id, int com)
{
    servo[id] = com;
    return TRUE;
}

int write_dio(unsigned short buf)
{
    return FALSE;
}

int open_iob(void)
{
    std::cout << "dummy IOB is opened" << std::endl;
    for (int i=0; i<number_of_joints(); i++){
        command[i] = 0.0;
        power[i] = OFF;
        servo[i] = OFF;
    }
    clock_gettime(CLOCK_MONOTONIC, &g_ts);
    return TRUE;
} 

int close_iob(void)
{
    std::cout << "dummy IOB is closed" << std::endl;
    return TRUE;
}

int reset_body(void)
{
    for (int i=0; i<number_of_joints(); i++){
        power[i] = servo[i] = OFF;
    }
    return TRUE;
}

int joint_calibration(int id, double angle)
{
    return FALSE;
}

int read_gyro_sensor_offset(int id, double *offset)
{
    for (int i=0; i<3; i++){
        offset[i] = gyro_offset[id][i];
    }
    return TRUE;
}

int write_gyro_sensor_offset(int id, double *offset)
{
    for (int i=0; i<3; i++){
        gyro_offset[id][i] = offset[i];
    }
    return TRUE;
}

int read_accelerometer_offset(int id, double *offset)
{
    for (int i=0; i<3; i++){
        offset[i] = accel_offset[id][i];
    }
    return TRUE;
}

int write_accelerometer_offset(int id, double *offset)
{
    for (int i=0; i<3; i++){
        accel_offset[id][i] = offset[i];
    }
    return TRUE;
}

int read_force_offset(int id, double *offsets)
{
    for (int i=0; i<6; i++){
        offsets[i] = force_offset[id][i];
    }
    return TRUE;
}

int write_force_offset(int id, double *offsets)
{
    for (int i=0; i<6; i++){
        force_offset[id][i] = offsets[i];
    }
    return TRUE;
}

int write_attitude_sensor_offset(int id, double *offset)
{
    return FALSE;
}

int read_calib_state(int id, int *s)
{
    CHECK_JOINT_ID(id);
    int v = id/2;
    *s = v%2==0 ? ON : OFF;
    return TRUE;
}

int lock_iob()
{
    if (isLocked) return FALSE;

    isLocked = true;
    return TRUE;
}
int unlock_iob()
{
    isLocked = false;
    return TRUE;
}

int read_lock_owner(pid_t *pid)
{
  return FALSE;
}

int read_limit_angle(int id, double *angle)
{
  return FALSE;
}

int read_angle_offset(int id, double *angle)
{
  return FALSE;
}

int write_angle_offset(int id, double angle)
{
  return FALSE;
}

int read_ulimit_angle(int id, double *angle)
{
  return FALSE;
}

int read_llimit_angle(int id, double *angle)
{
  return FALSE;
}

int write_ulimit_angle(int id, double angle)
{
  return FALSE;
}

int write_llimit_angle(int id, double angle)
{
  return FALSE;
}

int read_encoder_pulse(int id, double *ec)
{
  return FALSE;
}

int read_gear_ratio(int id, double *gr)
{
  return FALSE;
}

int read_torque_const(int id, double *tc)
{
  return FALSE;
}
int read_torque_limit(int id, double *limit)
{
  return FALSE;
}

unsigned long long read_iob_frame()
{
    ++frame;
    if (frame == 5) frame = 0;
    return frame;
}

int number_of_substeps()
{
    return 5;
}

int read_power(double *voltage, double *current)
{
    *voltage = ((double)random()-RAND_MAX/2)/(RAND_MAX/2)*1+48;
    *current = ((double)random()-RAND_MAX/2)/(RAND_MAX/2)*0.5+1;
    return TRUE;
}

#if defined(ROBOT_IOB_VERSION) && ROBOT_IOB_VERSION >= 2
int number_of_batteries()
{
    return 1;
}

int read_battery(int id, double *voltage, double *current, double *soc)
{
    *voltage = ((double)random()-RAND_MAX/2)/(RAND_MAX/2)*1+48;
    *current = ((double)random()-RAND_MAX/2)/(RAND_MAX/2)*0.5+1;
    *soc = ((double)random()-RAND_MAX/2)/(RAND_MAX/2)*0.5+50;
    return TRUE;
}

int number_of_thermometers()
{
    return 0;
}
#endif

#if defined(ROBOT_IOB_VERSION) && ROBOT_IOB_VERSION >= 3
int write_command_acceleration(int id, double acc)
{
    return FALSE;
}

int write_command_accelerations(const double *accs)
{
    return FALSE;
}

int write_joint_inertia(int id, double mn)
{
    return FALSE;
}

int write_joint_inertias(const double *mns)
{
    return FALSE;
}

int read_pd_controller_torques(double *torques)
{
    return FALSE;
}

int write_disturbance_observer(int com)
{
    return FALSE;
}

int write_disturbance_observer_gain(double gain)
{
    return FALSE;
}
#endif

#if defined(ROBOT_IOB_VERSION) && ROBOT_IOB_VERSION >= 4
int read_torque_pgain(int id, double *gain)
{
    return FALSE;
}

int write_torque_pgain(int id, double gain)
{
    return FALSE;
}

int read_torque_dgain(int id, double *gain)
{
    return FALSE;
}

int write_torque_dgain(int id, double gain)
{
    return FALSE;
}
#endif

int read_driver_temperature(int id, unsigned char *v)
{
    *v = id * 2;
    return TRUE;
}

void timespec_add_ns(timespec *ts, long ns)
{
    ts->tv_nsec += ns;
    while (ts->tv_nsec > 1e9){
        ts->tv_sec += 1;
        ts->tv_nsec -= 1e9;
    }
}

double timespec_compare(timespec *ts1, timespec *ts2)
{
    double dts = ts1->tv_sec - ts2->tv_sec;
    double dtn = ts1->tv_nsec - ts2->tv_nsec;
    return dts*1e9+dtn;
}

int wait_for_iob_signal()
{
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &g_ts, 0);
    timespec_add_ns(&g_ts, g_period_ns);
    timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    double dt = timespec_compare(&g_ts, &now);
    if (dt <= 0){
        //printf("overrun(%d[ms])\n", -dt*1e6);
        do {
            timespec_add_ns(&g_ts, g_period_ns);
        }while(timespec_compare(&g_ts, &now)<=0);
    }
    return 0;
}

size_t length_of_extra_servo_state(int id)
{
    return 0;
}

int read_extra_servo_state(int id, int *state)
{
    return TRUE;
}

int set_signal_period(long period_ns)
{
    g_period_ns = period_ns;
    return TRUE;
}

long get_signal_period()
{
    return g_period_ns;
}

int initializeJointAngle(const char *name, const char *option)
{
    sleep(3);
    return TRUE;
}

int read_digital_input(char *dinput)
{
    return FALSE;
}

int length_digital_input()
{
    return 0;
}

int write_digital_output(const char *doutput)
{
    return FALSE;
}

int write_digital_output_with_mask(const char *doutput, const char *mask)
{
    return FALSE;
}

int length_digital_output()
{
    return 0;
}

int read_digital_output(char *doutput)
{
    return FALSE;
}


