set(comp_sources ForwardKinematics.cpp ForwardKinematicsService_impl.cpp)
set(libs ${OPENHRP_LIBRARIES} hrpsysBaseStub)
add_library(ForwardKinematics SHARED ${comp_sources})
target_link_libraries(ForwardKinematics ${libs})
set_target_properties(ForwardKinematics PROPERTIES PREFIX "")

add_executable(ForwardKinematicsComp ForwardKinematicsComp.cpp ${comp_sources})
target_link_libraries(ForwardKinematicsComp ${libs})

set(target ForwardKinematics ForwardKinematicsComp)

install(TARGETS ${target}
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
)
