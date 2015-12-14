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
  configure_file(SampleRobot.conf.in ${CMAKE_CURRENT_BINARY_DIR}/SampleRobot.${_tmp_controller_period}.log.conf)
  file(APPEND ${CMAKE_CURRENT_BINARY_DIR}/SampleRobot.${_tmp_controller_period}.log.conf "exec_cxt.periodic.type: PeriodicExecutionContext\n")
  file(APPEND ${CMAKE_CURRENT_BINARY_DIR}/SampleRobot.${_tmp_controller_period}.log.conf "exec_cxt.periodic.rate: ${_tmp_controller_period}\n")
endforeach()
configure_file(SampleRobot.kinematicsonly.xml.in ${CMAKE_CURRENT_BINARY_DIR}/SampleRobot.kinematicsonly.xml)
configure_file(SampleRobot.carryobject.xml.in ${CMAKE_CURRENT_BINARY_DIR}/SampleRobot.carryobject.xml)

# for virtual force sensor testing
set(CONTROLLER_TIME 0.002)
set(CONTROLLER_PERIOD 500)
set(VirtualForceSensorSetting "virtual_force_sensor")
configure_file(SampleRobot.conf.in ${CMAKE_CURRENT_BINARY_DIR}/SampleRobot.500.vfs.conf)
set(VirtualForceSensorSetting "#virtual_force_sensor") # comment out
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

install(FILES 
  # files for 500[Hz] = 0.002[s]
  ${CMAKE_CURRENT_BINARY_DIR}/SampleRobot.500.conf
  ${CMAKE_CURRENT_BINARY_DIR}/SampleRobot.500.log.conf
  ${CMAKE_CURRENT_BINARY_DIR}/SampleRobot.500.xml
  ${CMAKE_CURRENT_BINARY_DIR}/SampleRobot.500.torque.xml
  ${CMAKE_CURRENT_BINARY_DIR}/SampleRobot.RobotHardware.500.conf
  # files for 200[Hz]  = 0.005[s]
  ${CMAKE_CURRENT_BINARY_DIR}/SampleRobot.200.conf
  ${CMAKE_CURRENT_BINARY_DIR}/SampleRobot.200.log.conf
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
  SampleRobot.DRCTestbed.xml
  ${CMAKE_CURRENT_BINARY_DIR}/SampleRobot.kinematicsonly.xml
  ${CMAKE_CURRENT_BINARY_DIR}/SampleRobot.carryobject.xml
  ForceSensorOffset_SampleRobot.txt
  DESTINATION share/hrpsys/samples/SampleRobot)

file(GLOB python_scripts RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.py)
install(PROGRAMS
    ${python_scripts}
  DESTINATION share/hrpsys/samples/SampleRobot)
add_subdirectory(rtc)
add_subdirectory(data)
