set(comp_sources ThermoEstimator.cpp)
set(libs hrpModel-${OPENHRP_LIBRARY_VERSION} hrpUtil-${OPENHRP_LIBRARY_VERSION} hrpsysBaseStub)
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
