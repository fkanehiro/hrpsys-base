configure_file(rtc.conf.in ${CMAKE_CURRENT_BINARY_DIR}/rtc.conf)

install(PROGRAMS
  HRP-4C.sh
  DESTINATION share/hrpsys/samples/HRP-4C)

install(FILES
  HRP-4C.xml
  ${CMAKE_CURRENT_BINARY_DIR}/rtc.conf
  HRP-4C.py
  DESTINATION share/hrpsys/samples/HRP-4C)

add_subdirectory(data)
  
