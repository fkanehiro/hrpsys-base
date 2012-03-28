generate_stub_skel(RobotHardwareService "OpenHRP")

set(comp_source  robot.cpp RobotHardware.cpp RobotHardwareService_impl.cpp RobotHardwareServiceSk.cpp)
add_library(RobotHardware SHARED ${comp_source} )
target_link_libraries(RobotHardware ${OPENRTM_LIBRARIES} hrpIo hrpModel-3.1 hrpCollision-3.1 hrpUtil-3.1)
set_target_properties(RobotHardware PROPERTIES PREFIX "")

add_executable(RobotHardwareComp RobotHardwareComp.cpp ${comp_source})
target_link_libraries(RobotHardwareComp ${OPENRTM_LIBRARIES} hrpIo hrpModel-3.1 hrpCollision-3.1 hrpUtil-3.1)

set(target RobotHardware RobotHardwareComp)

install(TARGETS ${target}
  RUNTIME DESTINATION bin CONFIGURATIONS Release Debug
  LIBRARY DESTINATION lib CONFIGURATIONS Release Debug
)

