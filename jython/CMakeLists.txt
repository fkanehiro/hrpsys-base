configure_file(hrpsyspy.in ${CMAKE_CURRENT_BINARY_DIR}/hrpsyspy)
configure_file(rtc.conf.in ${CMAKE_CURRENT_BINARY_DIR}/rtc.conf)

set(jarfile ${CMAKE_CURRENT_BINARY_DIR}/openrtm.jar)

add_custom_command(
  OUTPUT ${jarfile}
  COMMAND ${IDLJ} ${idlj_flags} -I. -I${OPENRTM_DIR}/include/rtm/idl ${OPENRTM_DIR}/include/rtm/idl/Manager.idl
  COMMAND ${IDLJ} ${idlj_flags} -I. -I${OPENRTM_DIR}/include/rtm/idl ${OPENRTM_DIR}/include/rtm/idl/OpenRTM.idl
  COMMAND ${IDLJ} ${idlj_flags} -I. -I${OPENRTM_DIR}/include/rtm/idl ${OPENRTM_DIR}/include/rtm/idl/DataPort.idl
  COMMAND ${JAVAC} ${javac_flags} src/*/*.java
  COMMAND ${JAR} cf ${jarfile} OpenRTM RTC RTM _SDOPackage
)

add_custom_target(OpenRTMStub ALL DEPENDS ${jarfile})

install(PROGRAMS ${CMAKE_CURRENT_BINARY_DIR}/hrpsyspy DESTINATION bin)
install(FILES ${jarfile} DESTINATION share/hrpsys/jar)
install(FILES rtm.py waitInput.py poseEditor.py DESTINATION share/hrpsys/jython)
install(FILES sample.py NullComponent.conf ${CMAKE_CURRENT_BINARY_DIR}/rtc.conf DESTINATION share/hrpsys/samples)
