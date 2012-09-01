configure_file(hrpsys.sh.in ${CMAKE_CURRENT_BINARY_DIR}/hrpsys.sh)
configure_file(rtc.conf.in ${CMAKE_CURRENT_BINARY_DIR}/rtc.conf)
configure_file(PA10.conf.in ${CMAKE_CURRENT_BINARY_DIR}/PA10.conf)
configure_file(PA10monitor.xml.in ${CMAKE_CURRENT_BINARY_DIR}/PA10monitor.xml)
configure_file(PA10simulation.xml.in ${CMAKE_CURRENT_BINARY_DIR}/PA10simulation.xml)
configure_file(RobotHardware.conf.in ${CMAKE_CURRENT_BINARY_DIR}/RobotHardware.conf)

install(PROGRAMS
  ${CMAKE_CURRENT_BINARY_DIR}/hrpsys.sh
  DESTINATION share/hrpsys/samples/PA10)

install(FILES 
  ${CMAKE_CURRENT_BINARY_DIR}/rtc.conf
  ${CMAKE_CURRENT_BINARY_DIR}/PA10.conf
  ${CMAKE_CURRENT_BINARY_DIR}/PA10monitor.xml
  ${CMAKE_CURRENT_BINARY_DIR}/PA10simulation.xml
  ${CMAKE_CURRENT_BINARY_DIR}/RobotHardware.conf
  PA10.py
  DESTINATION share/hrpsys/samples/PA10)

