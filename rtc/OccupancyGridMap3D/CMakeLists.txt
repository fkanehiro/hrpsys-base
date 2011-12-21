include_directories(${OCTOMAP_INCLUDE_DIR})
link_directories(${OCTOMAP_DIR}/lib)


generate_stub_skel(OGMap3DService)

set(comp_sources OccupancyGridMap3D.cpp OGMap3DService_impl.cpp OGMap3DServiceSk.cpp)
add_library(OccupancyGridMap3D SHARED ${comp_sources})
target_link_libraries(OccupancyGridMap3D ${OPENHRP_LIBRARIES} ${OCTOMAP_LIBRARIES} rtcpcl_pointcloud_type)
set_target_properties(OccupancyGridMap3D PROPERTIES PREFIX "")

add_executable(OccupancyGridMap3DComp OccupancyGridMap3DComp.cpp ${comp_sources})
target_link_libraries(OccupancyGridMap3DComp ${OPENHRP_LIBRARIES} ${OCTOMAP_LIBRARIES} rtcpcl_pointcloud_type)

set(target OccupancyGridMap3D OccupancyGridMap3DComp)

install(TARGETS ${target}
  RUNTIME DESTINATION bin CONFIGURATIONS Release Debug
  LIBRARY DESTINATION lib CONFIGURATIONS Release Debug
)
