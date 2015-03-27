# generate .conf and .xml files for two controller period
set(controller_time_list 0.002 0.005) # [s]
set(controller_period_list 500 200) # [Hz]
foreach(_idx 0 1)
  list(GET controller_time_list ${_idx} _tmp_controller_time)
  list(GET controller_period_list ${_idx} _tmp_controller_period)
  set(CONTROLLER_TIME ${_tmp_controller_time})
  set(CONTROLLER_PERIOD ${_tmp_controller_period})
  set(VirtualForceSensorSetting "#virtual_force_sensor") # comment out by default
  set(JointLimitTableSetting "#joint_limit_table") # comment out by default
  configure_file(SampleRobot.RobotHardware.conf.in ${CMAKE_CURRENT_BINARY_DIR}/SampleRobot.RobotHardware.${_tmp_controller_period}.conf)
  configure_file(SampleRobot.xml.in ${CMAKE_CURRENT_BINARY_DIR}/SampleRobot.${_tmp_controller_period}.xml)
  configure_file(SampleRobot.torque.xml.in ${CMAKE_CURRENT_BINARY_DIR}/SampleRobot.${_tmp_controller_period}.torque.xml)
  configure_file(SampleRobot.conf.in ${CMAKE_CURRENT_BINARY_DIR}/SampleRobot.${_tmp_controller_period}.conf)
endforeach()

# for virtual force sensor testing
set(CONTROLLER_TIME 0.002)
set(CONTROLLER_PERIOD 500)
set(VirtualForceSensorSetting "virtual_force_sensor")
configure_file(SampleRobot.conf.in ${CMAKE_CURRENT_BINARY_DIR}/SampleRobot.500.vfs.conf)
set(JointLimitTableSetting "joint_limit_table")
configure_file(SampleRobot.conf.in ${CMAKE_CURRENT_BINARY_DIR}/SampleRobot.500.el.conf)

set(robot_waist_translation "-0.8  0  0" "-4.15  -0.2  0" "-5.83  -0.2  -0.6096")
set(file_suffix "SlopeUpDown" "StairUp" "StairDown")
foreach(_idx 0 1 2)
  list(GET robot_waist_translation ${_idx} _tmp_robot_waist_translation)
  list(GET file_suffix ${_idx} _tmp_file_suffix)
  set(ROBOT_WAIST_TRANSLATION ${_tmp_robot_waist_translation})
  configure_file(SampleRobot.TerrainFloor.xml.in ${CMAKE_CURRENT_BINARY_DIR}/SampleRobot.TerrainFloor.${_tmp_file_suffix}.xml)
endforeach()

# Use configure_file to specify openhrp3 directory for sample1.wrl model
configure_file(samplerobot_walk.py.in ${CMAKE_CURRENT_BINARY_DIR}/samplerobot_walk.py)
configure_file(samplerobot_auto_balancer.py.in ${CMAKE_CURRENT_BINARY_DIR}/samplerobot_auto_balancer.py)
configure_file(samplerobot_terrain_walk.py.in ${CMAKE_CURRENT_BINARY_DIR}/samplerobot_terrain_walk.py)
configure_file(samplerobot_data_logger.py.in ${CMAKE_CURRENT_BINARY_DIR}/samplerobot_data_logger.py)
configure_file(samplerobot_impedance_controller.py.in ${CMAKE_CURRENT_BINARY_DIR}/samplerobot_impedance_controller.py)
configure_file(samplerobot_remove_force_offset.py.in ${CMAKE_CURRENT_BINARY_DIR}/samplerobot_remove_force_offset.py)
configure_file(samplerobot_stabilizer.py.in ${CMAKE_CURRENT_BINARY_DIR}/samplerobot_stabilizer.py)
configure_file(samplerobot_sequence_player.py.in ${CMAKE_CURRENT_BINARY_DIR}/samplerobot_sequence_player.py)
configure_file(samplerobot_collision_detector.py.in ${CMAKE_CURRENT_BINARY_DIR}/samplerobot_collision_detector.py)
configure_file(samplerobot_virtual_force_sensor.py.in ${CMAKE_CURRENT_BINARY_DIR}/samplerobot_virtual_force_sensor.py)
configure_file(samplerobot_soft_error_limiter.py.in ${CMAKE_CURRENT_BINARY_DIR}/samplerobot_soft_error_limiter.py)
configure_file(samplerobot_kalman_filter.py.in ${CMAKE_CURRENT_BINARY_DIR}/samplerobot_kalman_filter.py)

install(PROGRAMS
  ${CMAKE_CURRENT_BINARY_DIR}/samplerobot_data_logger.py
  ${CMAKE_CURRENT_BINARY_DIR}/samplerobot_auto_balancer.py
  ${CMAKE_CURRENT_BINARY_DIR}/samplerobot_terrain_walk.py
  ${CMAKE_CURRENT_BINARY_DIR}/samplerobot_stabilizer.py
  ${CMAKE_CURRENT_BINARY_DIR}/samplerobot_remove_force_offset.py
  ${CMAKE_CURRENT_BINARY_DIR}/samplerobot_impedance_controller.py
  ${CMAKE_CURRENT_BINARY_DIR}/samplerobot_walk.py
  ${CMAKE_CURRENT_BINARY_DIR}/samplerobot_sequence_player.py
  ${CMAKE_CURRENT_BINARY_DIR}/samplerobot_collision_detector.py
  ${CMAKE_CURRENT_BINARY_DIR}/samplerobot_virtual_force_sensor.py
  ${CMAKE_CURRENT_BINARY_DIR}/samplerobot_soft_error_limiter.py
  ${CMAKE_CURRENT_BINARY_DIR}/samplerobot_kalman_filter.py
  DESTINATION share/hrpsys/samples/SampleRobot)

install(FILES 
  # files for 500[Hz] = 0.002[s]
  ${CMAKE_CURRENT_BINARY_DIR}/SampleRobot.500.conf
  ${CMAKE_CURRENT_BINARY_DIR}/SampleRobot.500.xml
  ${CMAKE_CURRENT_BINARY_DIR}/SampleRobot.500.torque.xml
  ${CMAKE_CURRENT_BINARY_DIR}/SampleRobot.RobotHardware.500.conf
  # files for 200[Hz]  = 0.005[s]
  ${CMAKE_CURRENT_BINARY_DIR}/SampleRobot.200.conf
  ${CMAKE_CURRENT_BINARY_DIR}/SampleRobot.200.xml
  ${CMAKE_CURRENT_BINARY_DIR}/SampleRobot.200.torque.xml
  ${CMAKE_CURRENT_BINARY_DIR}/SampleRobot.RobotHardware.200.conf
  # for virtual force sensor
  ${CMAKE_CURRENT_BINARY_DIR}/SampleRobot.500.vfs.conf
  # for SoftErrorLimiter
  ${CMAKE_CURRENT_BINARY_DIR}/SampleRobot.500.el.conf
  SampleRobot.PDgain.dat
  # TerrainFloor
  ${CMAKE_CURRENT_BINARY_DIR}/SampleRobot.TerrainFloor.SlopeUpDown.xml
  ${CMAKE_CURRENT_BINARY_DIR}/SampleRobot.TerrainFloor.StairUp.xml
  ${CMAKE_CURRENT_BINARY_DIR}/SampleRobot.TerrainFloor.StairDown.xml
  DESTINATION share/hrpsys/samples/SampleRobot)

add_subdirectory(rtc)

