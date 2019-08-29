set(comp_sources CameraImageViewer.cpp)
set(libs hrpsysBaseStub ${OpenCV_LIBRARIES})
add_library(CameraImageViewer SHARED ${comp_sources})
target_link_libraries(CameraImageViewer ${libs})
set_target_properties(CameraImageViewer PROPERTIES PREFIX "")

add_executable(CameraImageViewerComp CameraImageViewerComp.cpp ${comp_sources})
target_link_libraries(CameraImageViewerComp ${libs})

set(target CameraImageViewer CameraImageViewerComp)

install(TARGETS ${target}
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
)
