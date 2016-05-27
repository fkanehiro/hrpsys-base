set(seq_dir ${PROJECT_SOURCE_DIR}/rtc/SequencePlayer)
set(comp_sources ${seq_dir}/interpolator.cpp EmergencyStopper.cpp EmergencyStopperService_impl.cpp ../SoftErrorLimiter/beep.cpp)
set(libs hrpModel-3.1 hrpUtil-3.1 hrpsysBaseStub)
include_directories(${seq_dir})
add_library(EmergencyStopper SHARED ${comp_sources})
target_link_libraries(EmergencyStopper ${libs})
set_target_properties(EmergencyStopper PROPERTIES PREFIX "")

add_executable(EmergencyStopperComp EmergencyStopperComp.cpp ${comp_sources})
target_link_libraries(EmergencyStopperComp ${libs})

set(target EmergencyStopper EmergencyStopperComp)

install(TARGETS ${target}
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
)
