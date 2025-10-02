macro(GENERATE_AND_INSTALL_EMERGENCY_BUTTON_CHECKER ROBOT_NAME HOSTNAME)
  execute_process(COMMAND pkg-config --variable=model ${ROBOT_NAME}
    OUTPUT_VARIABLE ROBOT_MODEL_DIR
    RESULT_VARIABLE RESULT
    OUTPUT_STRIP_TRAILING_WHITESPACE)
  if(NOT EXISTS ${ROBOT_MODEL_DIR})
    message(WARNING "${ROBOT_NAME} model directory(${ROBOT_MODEL_DIR}) is not found, skipping generation EmergencyButtonChecker script for this robot")
  else()
    string(TOUPPER ${HOSTNAME} HOSTNAME_UPPER)
    message(STATUS "Found ${ROBOT_NAME} model in ${ROBOT_MODEL_DIR}\n\tGenerate ${HOSTNAME_UPPER}EmergencyButtonChecker.{sh, desktop}")
    # This is needed to use ${HOSTNAME} with configure_file macro
    set(HOSTNAME ${HOSTNAME})
    configure_file(EmergencyButtonChecker.sh.in ${CMAKE_CURRENT_BINARY_DIR}/${HOSTNAME_UPPER}EmergencyButtonChecker.sh)
    configure_file(EmergencyButtonChecker.desktop.in ${CMAKE_CURRENT_BINARY_DIR}/${HOSTNAME_UPPER}EmergencyButtonChecker.desktop)

    install(PROGRAMS ${CMAKE_CURRENT_BINARY_DIR}/${HOSTNAME_UPPER}EmergencyButtonChecker.sh
            DESTINATION bin)
    install(FILES emergency_stop_button.png
            DESTINATION share/icons)
    install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${HOSTNAME_UPPER}EmergencyButtonChecker.desktop
      DESTINATION $ENV{HOME}/.local/share/applications)
  endif()
endmacro()

set(comp_sources SoftwareEmergency.cpp SoftwareEmergencyService_impl.cpp)
set(libs ${OPENHRP_LIBRARIES} hrpsysBaseStub)
add_library(SoftwareEmergency SHARED ${comp_sources})
target_link_libraries(SoftwareEmergency ${libs})
set_target_properties(SoftwareEmergency PROPERTIES PREFIX "")

add_executable(SoftwareEmergencyComp SoftwareEmergencyComp.cpp ${comp_sources})
target_link_libraries(SoftwareEmergencyComp ${libs})

if (NOT APPLE)
  find_package(USB REQUIRED)
  include_directories(${USB_INCLUDE_DIR})

  add_executable(usbio main.c usbio.c)
  target_link_libraries(usbio usb)

  add_executable(EmergencyButtonChecker EmergencyButtonChecker.cpp usbio.c)
  target_link_libraries(EmergencyButtonChecker ${libs} usb)

  set(target SoftwareEmergency SoftwareEmergencyComp EmergencyButtonChecker)

  GENERATE_AND_INSTALL_EMERGENCY_BUTTON_CHECKER(hrp2kai hrp2001c)
  GENERATE_AND_INSTALL_EMERGENCY_BUTTON_CHECKER(hrp2kai hrp2012c)
  GENERATE_AND_INSTALL_EMERGENCY_BUTTON_CHECKER(hrp5p hrp5p01c)
else()
  set(target SoftwareEmergency SoftwareEmergencyComp)
endif()

install(TARGETS ${target}
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
)

