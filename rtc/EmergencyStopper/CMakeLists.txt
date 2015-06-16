set(comp_sources EmergencyStopper.cpp EmergencyStopperService_impl.cpp)
set(libs hrpsysBaseStub)
add_library(EmergencyStopper SHARED ${comp_sources})
target_link_libraries(EmergencyStopper ${libs})
set_target_properties(EmergencyStopper PROPERTIES PREFIX "")

add_executable(EmergencyStopperComp EmergencyStopperComp.cpp ${comp_sources})
target_link_libraries(EmergencyStopperComp ${libs})

set(target EmergencyStopper EmergencyStopperComp)

install(TARGETS ${target}
  RUNTIME DESTINATION bin CONFIGURATIONS Release Debug
  LIBRARY DESTINATION lib CONFIGURATIONS Release Debug
)
