generate_stub_skel(ForwardKinematicsService "OpenHRP")

set(comp_sources ForwardKinematics.cpp ForwardKinematicsServiceSk.cpp ForwardKinematicsService_impl.cpp)
add_library(ForwardKinematics SHARED ${comp_sources})
target_link_libraries(ForwardKinematics ${OPENRTM_LIBRARIES} ${OPENHRP_LIBRARIES})
set_target_properties(ForwardKinematics PROPERTIES PREFIX "")

add_executable(ForwardKinematicsComp ForwardKinematicsComp.cpp ${comp_sources})
target_link_libraries(ForwardKinematicsComp ${OPENRTM_LIBRARIES} ${OPENHRP_LIBRARIES})

set(target ForwardKinematics ForwardKinematicsComp)

install(TARGETS ${target}
  RUNTIME DESTINATION bin CONFIGURATIONS Release Debug
  LIBRARY DESTINATION lib CONFIGURATIONS Release Debug
)
