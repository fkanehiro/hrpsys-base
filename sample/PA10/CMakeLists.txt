# shared object file suffix
if(APPLE)
  set(SO_SFX "dylib")
else()
  set(SO_SFX "so")
endif()

configure_file(hrpsys.sh.in ${CMAKE_CURRENT_BINARY_DIR}/hrpsys.sh)
configure_file(rtc.conf.robot.in ${CMAKE_CURRENT_BINARY_DIR}/rtc.conf.robot)
configure_file(PA10.conf.in ${CMAKE_CURRENT_BINARY_DIR}/PA10.conf)
configure_file(PA10.xml.in ${CMAKE_CURRENT_BINARY_DIR}/PA10.xml)

install(PROGRAMS
  ${CMAKE_CURRENT_BINARY_DIR}/hrpsys.sh
  DESTINATION share/hrpsys/samples/PA10)

install(FILES 
  ${CMAKE_CURRENT_BINARY_DIR}/rtc.conf.robot
  ${CMAKE_CURRENT_BINARY_DIR}/PA10.conf
  ${CMAKE_CURRENT_BINARY_DIR}/PA10.xml
  PA10.py
  DESTINATION share/hrpsys/samples/PA10)

