set(comp_source  robot.cpp RobotHardware.cpp RobotHardwareService_impl.cpp)
set(libs hrpIo hrpModel-3.1 hrpCollision-3.1 hrpUtil-3.1 hrpsysBaseStub)
link_directories(${LIBIO_DIR})

add_library(RobotHardware SHARED ${comp_source})
target_link_libraries(RobotHardware  ${libs})
set_target_properties(RobotHardware PROPERTIES PREFIX "")

add_executable(RobotHardwareComp RobotHardwareComp.cpp ${comp_source})
target_link_libraries(RobotHardwareComp ${libs})

set(target RobotHardware RobotHardwareComp)

install(TARGETS ${target}
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
)

