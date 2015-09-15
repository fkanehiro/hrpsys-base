#configure_file(Sample4LegRobot.RobotHardware.conf.in ${CMAKE_CURRENT_BINARY_DIR}/Sample4LegRobot.RobotHardware.${_tmp_controller_period}.conf)
configure_file(Sample4LegRobot.xml.in ${CMAKE_CURRENT_BINARY_DIR}/Sample4LegRobot.xml)
configure_file(Sample4LegRobot.torque.xml.in ${CMAKE_CURRENT_BINARY_DIR}/Sample4LegRobot.torque.xml)
configure_file(Sample4LegRobot.kinematicsonly.xml.in ${CMAKE_CURRENT_BINARY_DIR}/Sample4LegRobot.kinematicsonly.xml)
configure_file(Sample4LegRobot.conf.in ${CMAKE_CURRENT_BINARY_DIR}/Sample4LegRobot.conf)
configure_file(Sample4LegRobot.RobotHardware.conf.in ${CMAKE_CURRENT_BINARY_DIR}/Sample4LegRobot.RobotHardware.conf)

install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/Sample4LegRobot.conf
  ${CMAKE_CURRENT_BINARY_DIR}/Sample4LegRobot.RobotHardware.conf
  ${CMAKE_CURRENT_BINARY_DIR}/Sample4LegRobot.xml
  ${CMAKE_CURRENT_BINARY_DIR}/Sample4LegRobot.torque.xml
  ${CMAKE_CURRENT_BINARY_DIR}/Sample4LegRobot.kinematicsonly.xml
  Sample4LegRobot.PDgain.dat
  DESTINATION share/hrpsys/samples/Sample4LegRobot)

file(GLOB python_scripts RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.py)
install(PROGRAMS
  ${python_scripts}
  DESTINATION share/hrpsys/samples/Sample4LegRobot)

