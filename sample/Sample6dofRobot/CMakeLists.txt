configure_file(Sample6dofRobot.RobotHardware.conf.in ${CMAKE_CURRENT_BINARY_DIR}/Sample6dofRobot.RobotHardware.conf)
configure_file(Sample6dofRobot.xml.in ${CMAKE_CURRENT_BINARY_DIR}/Sample6dofRobot.xml)
configure_file(Sample6dofRobot.conf.in ${CMAKE_CURRENT_BINARY_DIR}/Sample6dofRobot.conf)

# Use configure_file to specify hrpsys install directory for sample6dofrobot model
configure_file(sample6dofrobot_kalman_filter.py.in ${CMAKE_CURRENT_BINARY_DIR}/sample6dofrobot_kalman_filter.py)

install(PROGRAMS
  ${CMAKE_CURRENT_BINARY_DIR}/sample6dofrobot_kalman_filter.py
  DESTINATION share/hrpsys/samples/Sample6dofRobot)

install(FILES 
  ${CMAKE_CURRENT_BINARY_DIR}/Sample6dofRobot.conf
  ${CMAKE_CURRENT_BINARY_DIR}/Sample6dofRobot.xml
  ${CMAKE_CURRENT_BINARY_DIR}/Sample6dofRobot.RobotHardware.conf
  Sample6dofRobot.wrl
  DESTINATION share/hrpsys/samples/Sample6dofRobot)

