set(comp_sources RemoveForceSensorLinkOffset.cpp RemoveForceSensorLinkOffsetService_impl.cpp ../ImpedanceController/RatsMatrix.cpp)
set(libs hrpModel-3.1 hrpUtil-3.1 hrpsysBaseStub)
add_library(RemoveForceSensorLinkOffset SHARED ${comp_sources})
target_link_libraries(RemoveForceSensorLinkOffset ${libs})
set_target_properties(RemoveForceSensorLinkOffset PROPERTIES PREFIX "")

add_executable(RemoveForceSensorLinkOffsetComp RemoveForceSensorLinkOffsetComp.cpp ${comp_sources})
target_link_libraries(RemoveForceSensorLinkOffsetComp ${libs})

set(target RemoveForceSensorLinkOffset RemoveForceSensorLinkOffsetComp)

install(TARGETS ${target}
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
)
