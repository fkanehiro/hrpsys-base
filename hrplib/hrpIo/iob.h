/**
 * @file iob.h
 * @brief abstract interface for the robot hardware
 */
#ifndef __IOB_H__
#define __IOB_H__

#define ON		1
#define OFF		0

#define MASK_ON		0
#define MASK_OFF	1

/**
 * @name return value
 */
//@{
#ifndef FALSE
#define FALSE		0
#endif

#ifndef TRUE
#define TRUE		1
#endif

#define E_ID	-1 ///< invalid joint(or sensor) id
//@}

/**
 * @name servo alarm
 */
//@{
#define SS_OVER_VOLTAGE		0x001
#define SS_OVER_LOAD		0x002
#define SS_OVER_VELOCITY	0x004
#define SS_OVER_CURRENT		0x008

#define SS_OVER_HEAT		0x010
#define SS_TORQUE_LIMIT		0x020
#define SS_VELOCITY_LIMIT	0x040
#define SS_FORWARD_LIMIT	0x080

#define SS_REVERSE_LIMIT	0x100
#define SS_POSITION_ERROR	0x200
#define SS_ENCODER_ERROR	0x400
#define SS_OTHER		0x800
//@}

#define JID_ALL -1
#define JID_INVALID -2

#ifdef __cplusplus
extern "C"{
#endif

    typedef enum {
        JCM_FREE, 	///< free
        JCM_POSITION, 	///< position control
        JCM_TORQUE,	///< torque control
        JCM_VELOCITY,   ///< velocity control
        JCM_NUM 
    } joint_control_mode;

    /**
     * @name the number of joints and sensors
     */
    // @{
    /**
     * @brief get the number of joints
     * @return the number of joints
     */
    int number_of_joints();

    /**
     * @brief set the number of joints
     * @param num the number of joints
     * @return TRUE if the number of joints is set, FALSE otherwise
     */
    int set_number_of_joints(int num);

    /**
     * @brief get the number of force sensors
     * @return the number of force sensors
     */
    int number_of_force_sensors();

    /**
     * @brief set the number of force sensors
     * @param num the number of force sensors
     * @return TRUE if the number of force sensors is set, FALSE otherwise
     */
    int set_number_of_force_sensors(int num);

    /**
     * @brief get the number of gyro sensors
     * @return the number of gyro sensors
     */
    int number_of_gyro_sensors();

    /**
     * @brief set the number of gyro sensors
     * @param num the number of gyro sensors
     * @return TRUE if the number of gyro sensors is set, FALSE otherwise
     */
    int set_number_of_gyro_sensors(int num);

    /**
     * @brief get the number of accelerometers
     * @return the number of accelerometers
     */
    int number_of_accelerometers();

    /**
     * @brief set the number of accelerometers
     * @param num the number of accelerometers
     * @return TRUE if the number of accelerometers is set, FALSE otherwise
     */
    int set_number_of_accelerometers(int num);

    /**
     * @brief get the number of attitude sensors
     * @return the number of attitude sensors
     */
    int number_of_attitude_sensors();

    // @}

    /**
     * @name joint angle
     */
    // @{
    /**
     * @brief read current joint angle[rad]
     * @param id	joint id
     * @param angle	actual joint angle[rad]
     * @retval TRUE this function is supported
     * @retval E_ID invalid joint id is specified
     * @retval FALSE otherwise
     */
    int read_actual_angle(int id, double *angle);

    /**
     * @brief read array of current joint angles[rad]
     * @param angles	array of joint angle[rad], length of array must be equal to number_of_joints()
     * @retval TRUE this function is supported
     * @retval FALSE otherwise
     */
    int read_actual_angles(double *angles);

    /**
     * @brief read offset value for joint[rad]
     * @param id	joint id
     * @param offset	offset value[rad]
     * @retval		TRUE offset value is read successfully
     * @retval		E_ID invalid id is specified
     * @retval		this function is not supported
     */
    int read_angle_offset(int id, double *offset);

    /**
     * @brief write offset value for joint[rad]
     * @param id	joint id
     * @param offset	offset value[rad]
     * @retval		TRUE offset values are written successfully
     * @retval		E_ID invalid id is specified
     * @retval		this function is not supported
     */
    int write_angle_offset(int id, double offset);

    //@}

    // @name joint
    // @{
    /**
     * @brief read power status of motor driver
     * @param id 	joint id
     * @param s		ON or OFF is returned
     * @retval TRUE	this function is supported
     * @retval E_ID invalid joint id is specified
     * @retval FALSE 	otherwise
     */
    int read_power_state(int id, int *s);

    /**
     * @brief turn on/off power supply for motor driver
     * @param id 	joint id
     * @param com	ON/OFF
     * @retval TRUE this function is supported
     * @retval E_ID invalid joint id is specified
     * @retval FALSE otherwise
     */
    int write_power_command(int id, int com);

    /**
     * @brief turn on/off power supply for motor driver
     * @param id 	joint id
     * @param com	ON/OFF
     * @retval TRUE this function is supported
     * @retval E_ID invalid joint id is specified
     * @retval FALSE otherwise
     */
    int read_power_command(int id, int *com);

    /**
     * @brief read servo status 
     * @param id	joint id
     * @param s		ON/OFF
     * @retval TRUE this function is supported
     * @retval E_ID invalid joint id is specified
     * @retval FALSE otherwise
     */
    int read_servo_state(int id, int *s);

    /**
     * @brief read servo alarms
     * @param id  	joint id
     * @param a 	servo alarms 
     * @retval TRUE this function is supported
     * @retval E_ID invalid joint id is specified
     * @retval FALSE otherwise
     */
    int read_servo_alarm(int id, int *a);
    
    /**
     * @brief read joint control mode
     * @param id 	joint id
     * @param s		joint control mode
     * @retval TRUE this function is supported
     * @retval E_ID invalid joint id is specified
     * @retval FALSE otherwise
     */
    int read_control_mode(int id, joint_control_mode *s);

    /**
     * @brief write joint control mode
     * @param id 	joint id
     * @param s		joint control mode
     * @retval TRUE this function is supported
     * @retval E_ID invalid joint id is specified
     * @retval FALSE otherwise
     */
    int write_control_mode(int id, joint_control_mode s);

    /**
     * @brief read array of current joint torques[Nm]
     * @param torques	array of actual joint torque[Nm], length of array must be equal to number_of_joints()
     * @retval TRUE this function is supported
     * @retval FALSE otherwise
     */
    int read_actual_torques(double *torques);

    /**
     * @brief read command torque[Nm]
     * @param id	joint id
     * @param torque	joint torque[Nm]
     * @retval TRUE this function is supported
     * @retval E_ID invalid joint id is specified
     * @retval FALSE otherwise
     */
    int read_command_torque(int id, double *torque);

    /**
     * @brief write command torque[Nm]
     * @param id	joint id
     * @param torque	joint torque[Nm]
     * @return		TRUE if this function is supported, FALSE otherwise
     */
    int write_command_torque(int id, double torque);

    /**
     * @brief read array of command torques[Nm]
     * @param torques	array of command torques[Nm]
     * @retval TRUE this function is supported
     * @retval FALSE otherwise
     */
    int read_command_torques(double *torques);

    /**
     * @brief read command angle[rad]
     * @param id	joint id
     * @param angle	command joint angle[rad]
     * @retval TRUE this function is supported
     * @retval FALSE otherwise
     */
    int read_command_angle(int id, double *angle);

    /**
     * @brief write command angle[rad]
     * @param id	joint id
     * @param angle	command joint angle[rad]
     * @return		TRUE or E_ID
     */
    int write_command_angle(int id, double angle);

    /**
     * @brief read array of command angles[rad]
     * @param angles	array of joint angles[rad], length of array must equal to DOF
     * @retval TRUE this function is supported
     * @retval FALSE otherwise
     */
    int read_command_angles(double *angles);

    /**
     * @brief write array of command angles[rad]
     * @param angles	array of joint angles[rad], length of array must equal to DOF
     * @retval TRUE this function is supported
     * @retval FALSE otherwise
     */
    int write_command_angles(const double *angles);

    /**
     * @brief read P gain[Nm/rad]
     * @param id	joint id
     * @param gain	P gain[Nm/rad]
     * @return		TRUE or E_ID
     */
    int read_pgain(int id, double *gain);

    /**
     * @brief write P gain[Nm/rad]
     * @param id	joint id
     * @param gain	P gain[Nm/rad]
     * @return		TRUE or E_ID
     */
    int write_pgain(int id, double gain);

    /**
     * @brief read D gain[Nm/(rad/s)]
     * @param id	joint id
     * @param gain	D gain[Nm/(rad/s)]
     * @return		TRUE or E_ID
     */
    int read_dgain(int id, double *gain);

    /**
     * @brief write D gain[Nm/(rad/s)]
     * @param id	joint id
     * @param gain	D gain[Nm/(rad/s)]
     * @return		TRUE or E_ID
     */
    int write_dgain(int id, double gain);

    /**
     * @brief read command angular velocity[rad/s]
     * @param id	joint id
     * @param vel	angular velocity [rad/s]
     * @return		TRUE or E_ID
     */
    int read_command_velocity(int id, double *vel);

    /**
     * @brief write command angular velocity[rad/s]
     * @param id	joint id
     * @param vel	angular velocity [rad/s]
     * @return		TRUE or E_ID
     */
    int write_command_velocity(int id, double vel);

    /**
     * @brief read command angular velocities[rad/s]
     * @param vels	array of angular velocity [rad/s]
     * @retval TRUE this function is supported
     * @retval FALSE otherwise
     */
    int read_command_velocities(double *vels);

    /**
     * @brief write command angular velocities[rad/s]
     * @param vels	array of angular velocity [rad/s]
     * @retval TRUE this function is supported
     * @retval FALSE otherwise
     */
    int write_command_velocities(double *vels);

    /**
     * @brief turn on/off joint servo
     * @param id	joint id
     * @param com 	ON/OFF
     * @return		TRUE if this function is supported, FALSE otherwise
     */
    int write_servo(int id, int com);

    /**
     * @brief read temperature of motor driver[Celsius]
     * @param id 	joint id
     * @param v 	temperature[Celsius]
     * @retval TRUE 	temperature is read successfully
     * @retval E_ID	invalid joint id is specified
     * @retval FALSE 	this function is not supported
     */
    int read_driver_temperature(int id, unsigned char* v);

    /**
     * @brief read callibration state of joint
     * @param id 	joint id
     * @param s		TRUE if calibration is already done, FALSE otherwise
     * @retval TRUE 	calibration status is read successfully
     * @retval E_ID	invalid joint id is specified
     * @retval FALSE 	this function is not supported
     */
    int read_calib_state(int id, int *s);

    //@}

    /**
     * @name force sensor
     */
    //@{
    /**
     * @brief read output of force sensor
     * @param id	Force Sensor id
     * @param forces	array of forces[N] and moments[Nm], length of array must be 6
     * @return		TRUE or E_ID
     */
    int read_force_sensor(int id, double *forces);

    /**
     * @brief read offset values for force sensor output
     * @param id	force/torque sensor id
     * @param offsets	offset values[N][Nm], length of array must be 6.
     * @retval		TRUE offset values are read successfully
     * @retval		E_ID invalid id is specified
     * @retval		this function is not supported
     */
    int read_force_offset(int id, double *offsets);

    /**
     * @brief write offset values for forcesensor output
     * @param id	accelerometer id
     * @param offsets	offset values[N][Nm], length of array must be 6.
     * @retval		TRUE offset values are written successfully
     * @retval		E_ID invalid id is specified
     * @retval		this function is not supported
     */
    int write_force_offset(int id, double *offsets);
    //@}

    /**
     * @name gyro sensor
     */
    //@{
    /**
     * @brief read output of gyro sensor
     * @param id	gyro sensor id
     * @param rates	angular velocities [rad/s], length of array must be 3
     * @return		TRUE or E_ID
     */
    int read_gyro_sensor(int id, double *rates);

    /**
     * @brief read offset values for gyro sensor output
     * @param id	gyro sensor id
     * @param offset	offset values[rad/s], length of array must be 3.
     * @retval		TRUE offset values are read successfully
     * @retval		E_ID invalid id is specified
     * @retval		this function is not supported
     */
    int read_gyro_sensor_offset(int id, double *offset);

    /**
     * @brief write offset values for gyro sensor output
     * @param id	gyro sensor id
     * @param offset	offset values[rad/s], length of array must be 3.
     * @retval		TRUE offset values are written successfully
     * @retval		E_ID invalid id is specified
     * @retval		this function is not supported
     */
    int write_gyro_sensor_offset(int id, double *offset);

    //@}

    /**
     * @name acceleromter
     */
    //@{
    /**
     * @brief read output of accelerometer
     * @param id	accelerometer id
     * @param accels	accelerations [m/s^2], length of array must be 3
     * @return		TRUE or E_ID
     */
    int read_accelerometer(int id, double *accels);

    /**
     * @brief read offset values for accelerometer output
     * @param id	accelerometer id
     * @param offset	offset values[rad/s^2], length of array must be 3.
     * @retval		TRUE offset values are read successfully
     * @retval		E_ID invalid id is specified
     * @retval		this function is not supported
     */
    int read_accelerometer_offset(int id, double *offset);

    /**
     * @brief write offset values for accelerometer output
     * @param id	accelerometer id
     * @param offset	offset values[rad/s^2], length of array must be 3.
     * @retval		TRUE offset values are written successfully
     * @retval		E_ID invalid id is specified
     * @retval		this function is not supported
     */
    int write_accelerometer_offset(int id, double *offset);

    //@}

    /**
     * @name attitude sensor
     */
    //@{
    /**
     * @brief read output of attitude sensor
     * @param id	attitude sensor id
     * @param att	roll-pitch-yaw angle[rad], length of array must be 3
     * @retval		TRUE sensor values are read successfully
     * @retval		E_ID invalid id is specified
     * @retval		this function is not supported
     */
    int read_attitude_sensor(int id, double *att);

    int write_attitude_sensor_offset(int id, double *offset);
    //@}

    /**
     * @name power supply
     */
    //@{
    /**
     * @brief		read status of power source
     * @param v		voltage[V]
     * @param a		current[A]
     * @return		TRUE or FALSE
     */
    int read_power(double *v, double *a);
    //@}

    /**
     * @name thermometer
     */
    //@{
    /**
     * @brief		read thermometer
     * @param id	id of thermometer
     * @param v		temperature[Celsius]
     * @retval		TRUE temperature is read successfully
     * @retval		E_ID invalid thermometer id is specified
     * @retval		this function is not supported
     */
    int read_temperature(int id, double *v);
    //@}

    /**
     * @name open/close 
     */
    //@{
    /**
     * @brief open connection with joint servo process
     * @retval TRUE opened successfully
     * @retval FALSE otherwise  
     */
    int open_iob(void);

    /**
     * @brief close connection with joint servo process
     * @retval TRUE closed successfully
     * @retval FALSE otherwise  
     */
    int close_iob(void);

    int reset_body(void);

    /**
     * @brief lock access to iob
     * @retval TRUE iob is locked successfully
     * @retval FALSE some other process is locking iob
     */
    int lock_iob();

    /**
     * @brief unlock access to iob
     */
    int unlock_iob();

    /**
     * @brief read id of the process whic is locking access to iob
     */
    int read_lock_owner(pid_t *pid);

    /**
     * @brief
     */
    int read_iob_frame();

    /**
     * @brief
     * @return the number of substeps
     */
    int number_of_substeps(); 
    //@}

  
    int wait_for_iob_signal();

#ifdef __cplusplus
}
#endif

#endif
