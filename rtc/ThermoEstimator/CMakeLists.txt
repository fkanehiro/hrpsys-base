set(comp_sources ThermoEstimator.cpp)
set(libs hrpModel-3.1 hrpUtil-3.1 hrpsysBaseStub)
add_library(ThermoEstimator SHARED ${comp_sources})
target_link_libraries(ThermoEstimator ${libs})
set_target_properties(ThermoEstimator PROPERTIES PREFIX "")

add_executable(ThermoEstimatorComp ThermoEstimatorComp.cpp ${comp_sources})
target_link_libraries(ThermoEstimatorComp ${libs})

set(target ThermoEstimator ThermoEstimatorComp)

install(TARGETS ${target}
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
)
