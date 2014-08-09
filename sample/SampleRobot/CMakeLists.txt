# generate .conf and .xml files for two controller period
set(controller_time_list 0.002 0.005) # [s]
set(controller_period_list 500 200) # [Hz]
foreach(_idx 0 1)
  list(GET controller_time_list ${_idx} _tmp_controller_time)
  list(GET controller_period_list ${_idx} _tmp_controller_period)
  set(CONTROLLER_TIME ${_tmp_controller_time})
  set(CONTROLLER_PERIOD ${_tmp_controller_period})
  configure_file(SampleRobot.RobotHardware.conf.in ${CMAKE_CURRENT_BINARY_DIR}/SampleRobot.RobotHardware.${_tmp_controller_period}.conf)
  configure_file(SampleRobot.xml.in ${CMAKE_CURRENT_BINARY_DIR}/SampleRobot.${_tmp_controller_period}.xml)
  configure_file(SampleRobot.torque.xml.in ${CMAKE_CURRENT_BINARY_DIR}/SampleRobot.${_tmp_controller_period}.torque.xml)
  configure_file(SampleRobot.conf.in ${CMAKE_CURRENT_BINARY_DIR}/SampleRobot.${_tmp_controller_period}.conf)
endforeach()

# Use configure_file to specify openhrp3 directory for sample1.wrl model
configure_file(samplerobot_walk.py.in ${CMAKE_CURRENT_BINARY_DIR}/samplerobot_walk.py)
configure_file(samplerobot_auto_balancer.py.in ${CMAKE_CURRENT_BINARY_DIR}/samplerobot_auto_balancer.py)
configure_file(samplerobot_data_logger.py.in ${CMAKE_CURRENT_BINARY_DIR}/samplerobot_data_logger.py)
configure_file(samplerobot_impedance_controller.py.in ${CMAKE_CURRENT_BINARY_DIR}/samplerobot_impedance_controller.py)
configure_file(samplerobot_remove_force_offset.py.in ${CMAKE_CURRENT_BINARY_DIR}/samplerobot_remove_force_offset.py)
configure_file(samplerobot_stabilizer.py.in ${CMAKE_CURRENT_BINARY_DIR}/samplerobot_stabilizer.py)

install(PROGRAMS
  ${CMAKE_CURRENT_BINARY_DIR}/samplerobot_data_logger.py
  ${CMAKE_CURRENT_BINARY_DIR}/samplerobot_auto_balancer.py
  ${CMAKE_CURRENT_BINARY_DIR}/samplerobot_stabilizer.py
  ${CMAKE_CURRENT_BINARY_DIR}/samplerobot_remove_force_offset.py
  ${CMAKE_CURRENT_BINARY_DIR}/samplerobot_impedance_controller.py
  ${CMAKE_CURRENT_BINARY_DIR}/samplerobot_walk.py
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
  SampleRobot.PDgain.dat
  DESTINATION share/hrpsys/samples/SampleRobot)

