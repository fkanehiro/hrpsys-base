set(comp_sources CameraImageLoader.cpp)
set(libs hrpsysBaseStub ${OpenCV_LIBRARIES})
add_library(CameraImageLoader SHARED ${comp_sources})
target_link_libraries(CameraImageLoader ${libs})
set_target_properties(CameraImageLoader PROPERTIES PREFIX "")

add_executable(CameraImageLoaderComp CameraImageLoaderComp.cpp ${comp_sources})
target_link_libraries(CameraImageLoaderComp ${libs})

set(target CameraImageLoader CameraImageLoaderComp)

install(TARGETS ${target}
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
)
