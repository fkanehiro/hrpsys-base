configure_file(HRP-4C.xml.in ${CMAKE_CURRENT_BINARY_DIR}/HRP-4C.xml)

install(PROGRAMS
  HRP-4C.sh
  DESTINATION share/hrpsys/samples/HRP-4C)

install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/HRP-4C.xml
  HRP-4C.py
  DESTINATION share/hrpsys/samples/HRP-4C)
