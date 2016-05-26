set(comp_sources Range2PointCloud.cpp)
set(libs hrpsysBaseStub ${OPENHRP_LIBRARIES})
add_library(Range2PointCloud SHARED ${comp_sources})
target_link_libraries(Range2PointCloud ${libs})
set_target_properties(Range2PointCloud PROPERTIES PREFIX "")

add_executable(Range2PointCloudComp Range2PointCloudComp.cpp ${comp_sources})
target_link_libraries(Range2PointCloudComp ${libs})

set(target Range2PointCloud Range2PointCloudComp)

install(TARGETS ${target}
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
)
