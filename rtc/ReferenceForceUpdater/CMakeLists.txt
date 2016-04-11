set(comp_sources ReferenceForceUpdater.cpp ReferenceForceUpdaterService_impl.cpp
  ../ImpedanceController/JointPathEx.cpp ../ImpedanceController/RatsMatrix.cpp ../SequencePlayer/interpolator.cpp)
set(libs hrpModel-3.1 hrpCollision-3.1 hrpUtil-3.1 hrpsysBaseStub)
add_library(ReferenceForceUpdater SHARED ${comp_sources})
target_link_libraries(ReferenceForceUpdater ${libs})
set_target_properties(ReferenceForceUpdater PROPERTIES PREFIX "")

add_executable(ReferenceForceUpdaterComp ReferenceForceUpdaterComp.cpp ${comp_sources})
target_link_libraries(ReferenceForceUpdaterComp ${libs})

set(target ReferenceForceUpdater ReferenceForceUpdaterComp)

install(TARGETS ${target}
  RUNTIME DESTINATION bin CONFIGURATIONS Release Debug
  LIBRARY DESTINATION lib CONFIGURATIONS Release Debug
)
