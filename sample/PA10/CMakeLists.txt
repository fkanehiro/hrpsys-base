configure_file(hrpsys.sh.in ${CMAKE_CURRENT_BINARY_DIR}/hrpsys.sh)
configure_file(rtc.conf.robot.in ${CMAKE_CURRENT_BINARY_DIR}/rtc.conf.robot)
configure_file(PA10.conf.in ${CMAKE_CURRENT_BINARY_DIR}/PA10.conf)
configure_file(PA10.xml.in ${CMAKE_CURRENT_BINARY_DIR}/PA10.xml)
configure_file(RobotHardware.conf.in ${CMAKE_CURRENT_BINARY_DIR}/RobotHardware.conf)

install(PROGRAMS
  ${CMAKE_CURRENT_BINARY_DIR}/hrpsys.sh
  DESTINATION share/hrpsys/samples/PA10)

install(FILES 
  ${CMAKE_CURRENT_BINARY_DIR}/rtc.conf.robot
  ${CMAKE_CURRENT_BINARY_DIR}/PA10.conf
  ${CMAKE_CURRENT_BINARY_DIR}/PA10.xml
  ${CMAKE_CURRENT_BINARY_DIR}/RobotHardware.conf
  PA10.py
  DESTINATION share/hrpsys/samples/PA10)

