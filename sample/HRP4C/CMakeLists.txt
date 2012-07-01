configure_file(rtc.conf.in ${CMAKE_CURRENT_BINARY_DIR}/rtc.conf)
configure_file(HRP4C.conf.in ${CMAKE_CURRENT_BINARY_DIR}/HRP4C.conf)

install(PROGRAMS
  HRP4C.sh
  DESTINATION share/hrpsys/samples/HRP4C)

install(FILES
  HRP4C.xml
  ${CMAKE_CURRENT_BINARY_DIR}/rtc.conf
  HRP4C.py
  ${CMAKE_CURRENT_BINARY_DIR}/HRP4C.conf
  DESTINATION share/hrpsys/samples/HRP4C)

add_subdirectory(data)
  
