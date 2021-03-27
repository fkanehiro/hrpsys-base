set(comp_sources AccelerationFilter.cpp AccelerationFilterService_impl.cpp ../TorqueFilter/IIRFilter.cpp)

set(libs hrpModel-${OPENHRP_LIBRARY_VERSION} hrpCollision-${OPENHRP_LIBRARY_VERSION} hrpUtil-${OPENHRP_LIBRARY_VERSION} hrpsysBaseStub)
add_library(AccelerationFilter SHARED ${comp_sources})
target_link_libraries(AccelerationFilter ${libs})
set_target_properties(AccelerationFilter PROPERTIES PREFIX "")

add_executable(AccelerationFilterComp AccelerationFilterComp.cpp ${comp_sources})
target_link_libraries(AccelerationFilterComp ${libs})

include_directories(${PROJECT_SOURCE_DIR}/rtc/SequencePlayer)

set(target AccelerationFilter AccelerationFilterComp)

install(TARGETS ${target}
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
)
