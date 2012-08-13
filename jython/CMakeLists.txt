configure_file(hrpsysjy.in ${CMAKE_CURRENT_BINARY_DIR}/hrpsysjy)
configure_file(rtc.conf.in ${CMAKE_CURRENT_BINARY_DIR}/rtc.conf)

set(jarfile ${CMAKE_CURRENT_BINARY_DIR}/openrtm.jar)

add_custom_command(
  OUTPUT ${jarfile}
  COMMAND ${IDLJ} ${idlj_flags} -I. -I${OPENRTM_IDL_DIR} ${OPENRTM_IDL_DIR}/Manager.idl
  COMMAND ${IDLJ} ${idlj_flags} -I. -I${OPENRTM_IDL_DIR} ${OPENRTM_IDL_DIR}/OpenRTM.idl
  COMMAND ${IDLJ} ${idlj_flags} -I. -I${OPENRTM_IDL_DIR} ${OPENRTM_IDL_DIR}/DataPort.idl
  COMMAND ${IDLJ} ${idlj_flags} -I. -I${OPENRTM_IDL_DIR} ${OPENRTM_IDL_DIR}/InterfaceDataTypes.idl
  COMMAND ${JAVAC} ${javac_flags} src/*/*.java
  COMMAND ${JAR} cf ${jarfile} OpenRTM RTC RTM _SDOPackage
)

add_custom_target(OpenRTMStub ALL DEPENDS ${jarfile})

install(PROGRAMS ${CMAKE_CURRENT_BINARY_DIR}/hrpsysjy DESTINATION bin)
install(FILES ${jarfile} DESTINATION share/java)
install(FILES rtm.py waitInput.py poseEditor.py DESTINATION share/hrpsys/jython)
install(FILES sample.py NullComponent.conf ${CMAKE_CURRENT_BINARY_DIR}/rtc.conf DESTINATION share/hrpsys/samples)
