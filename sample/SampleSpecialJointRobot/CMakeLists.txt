#configure_file(SampleSpecialJointRobot.RobotHardware.conf.in ${CMAKE_CURRENT_BINARY_DIR}/SampleSpecialJointRobot.RobotHardware.${_tmp_controller_period}.conf)
configure_file(SampleSpecialJointRobot.xml.in ${CMAKE_CURRENT_BINARY_DIR}/SampleSpecialJointRobot.xml)
configure_file(SampleSpecialJointRobot.torque.xml.in ${CMAKE_CURRENT_BINARY_DIR}/SampleSpecialJointRobot.torque.xml)
configure_file(SampleSpecialJointRobot.kinematicsonly.xml.in ${CMAKE_CURRENT_BINARY_DIR}/SampleSpecialJointRobot.kinematicsonly.xml)
configure_file(SampleSpecialJointRobot.conf.in ${CMAKE_CURRENT_BINARY_DIR}/SampleSpecialJointRobot.conf)
configure_file(SampleSpecialJointRobot.RobotHardware.conf.in ${CMAKE_CURRENT_BINARY_DIR}/SampleSpecialJointRobot.RobotHardware.conf)

install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/SampleSpecialJointRobot.conf
  ${CMAKE_CURRENT_BINARY_DIR}/SampleSpecialJointRobot.RobotHardware.conf
  ${CMAKE_CURRENT_BINARY_DIR}/SampleSpecialJointRobot.xml
  ${CMAKE_CURRENT_BINARY_DIR}/SampleSpecialJointRobot.torque.xml
  ${CMAKE_CURRENT_BINARY_DIR}/SampleSpecialJointRobot.kinematicsonly.xml
  SampleSpecialJointRobot.PDgain.dat
  DESTINATION share/hrpsys/samples/SampleSpecialJointRobot)

file(GLOB python_scripts RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.py)
install(PROGRAMS
  ${python_scripts}
  DESTINATION share/hrpsys/samples/SampleSpecialJointRobot)

