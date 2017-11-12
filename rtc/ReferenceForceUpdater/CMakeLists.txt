set(comp_sources ReferenceForceUpdater.cpp ReferenceForceUpdaterService_impl.cpp
  ../ImpedanceController/JointPathEx.cpp ../ImpedanceController/RatsMatrix.cpp ../SequencePlayer/interpolator.cpp)
set(libs hrpModel-${OPENHRP_LIBRARY_VERSION} hrpCollision-${OPENHRP_LIBRARY_VERSION} hrpUtil-${OPENHRP_LIBRARY_VERSION} hrpsysBaseStub)
add_library(ReferenceForceUpdater SHARED ${comp_sources})
target_link_libraries(ReferenceForceUpdater ${libs})
set_target_properties(ReferenceForceUpdater PROPERTIES PREFIX "")

add_executable(ReferenceForceUpdaterComp ReferenceForceUpdaterComp.cpp ${comp_sources})
target_link_libraries(ReferenceForceUpdaterComp ${libs})

set(target ReferenceForceUpdater ReferenceForceUpdaterComp)

install(TARGETS ${target}
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
)
