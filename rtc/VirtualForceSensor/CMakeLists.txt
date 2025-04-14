set(comp_sources VirtualForceSensor.cpp VirtualForceSensorService_impl.cpp)
set(libs hrpModel-${OPENHRP_LIBRARY_VERSION} hrpUtil-${OPENHRP_LIBRARY_VERSION} hrpsysBaseStub)
add_library(VirtualForceSensor SHARED ${comp_sources})
target_link_libraries(VirtualForceSensor ${libs})
set_target_properties(VirtualForceSensor PROPERTIES PREFIX "")

add_executable(VirtualForceSensorComp VirtualForceSensorComp.cpp ${comp_sources})
target_link_libraries(VirtualForceSensorComp ${libs})

set(target VirtualForceSensor VirtualForceSensorComp)

install(TARGETS ${target}
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
)
