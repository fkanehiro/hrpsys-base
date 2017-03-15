include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(comp_sources PointCloudViewer.cpp)
set(libs hrpsysBaseStub ${PCL_LIBRARIES})
add_library(PointCloudViewer SHARED ${comp_sources})
target_link_libraries(PointCloudViewer ${libs})
set_target_properties(PointCloudViewer PROPERTIES PREFIX "")

add_executable(PointCloudViewerComp PointCloudViewerComp.cpp ${comp_sources})
target_link_libraries(PointCloudViewerComp ${libs})

set(target PointCloudViewer PointCloudViewerComp)

install(TARGETS ${target}
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
)
