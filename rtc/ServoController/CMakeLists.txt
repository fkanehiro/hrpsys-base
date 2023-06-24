set(comp_sources ServoController.cpp ServoControllerService_impl.cpp)
set(libs hrpModel-${OPENHRP_LIBRARY_VERSION} hrpCollision-${OPENHRP_LIBRARY_VERSION} hrpUtil-${OPENHRP_LIBRARY_VERSION} hrpsysBaseStub)
add_library(ServoController SHARED ${comp_sources})
target_link_libraries(ServoController ${libs})
set_target_properties(ServoController PROPERTIES PREFIX "")

add_executable(ServoControllerComp ServoControllerComp.cpp ${comp_sources})
target_link_libraries(ServoControllerComp ${libs})

add_executable(testServoSerial testServoSerial.cpp)

set(target ServoController ServoControllerComp)

install(TARGETS ${target}
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
)
