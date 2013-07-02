set(comp_sources AbsoluteForceSensor.cpp AbsoluteForceSensorService_impl.cpp ../ImpedanceController/RatsMatrix.cpp)
set(libs hrpModel-3.1 hrpUtil-3.1 hrpsysBaseStub)
add_library(AbsoluteForceSensor SHARED ${comp_sources})
target_link_libraries(AbsoluteForceSensor ${libs})
set_target_properties(AbsoluteForceSensor PROPERTIES PREFIX "")

add_executable(AbsoluteForceSensorComp AbsoluteForceSensorComp.cpp ${comp_sources})
target_link_libraries(AbsoluteForceSensorComp ${libs})

set(target AbsoluteForceSensor AbsoluteForceSensorComp)

install(TARGETS ${target}
  RUNTIME DESTINATION bin CONFIGURATIONS Release Debug
  LIBRARY DESTINATION lib CONFIGURATIONS Release Debug
)
