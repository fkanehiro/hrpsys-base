configure_file(rtc.conf.in ${CMAKE_CURRENT_BINARY_DIR}/rtc.conf)
configure_file(HRP-4C.conf.in ${CMAKE_CURRENT_BINARY_DIR}/HRP-4C.conf)

install(PROGRAMS
  HRP-4C.sh
  DESTINATION share/hrpsys/samples/HRP-4C)

install(FILES
  HRP-4C.xml
  ${CMAKE_CURRENT_BINARY_DIR}/rtc.conf
  HRP-4C.py
  ${CMAKE_CURRENT_BINARY_DIR}/HRP-4C.conf
  DESTINATION share/hrpsys/samples/HRP-4C)

add_subdirectory(data)
  
