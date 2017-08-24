set(comp_sources CameraImageSaver.cpp)
set(libs hrpsysBaseStub ${OpenCV_LIBRARIES})
add_library(CameraImageSaver SHARED ${comp_sources})
target_link_libraries(CameraImageSaver ${libs})
set_target_properties(CameraImageSaver PROPERTIES PREFIX "")

add_executable(CameraImageSaverComp CameraImageSaverComp.cpp ${comp_sources})
target_link_libraries(CameraImageSaverComp ${libs})

set(target CameraImageSaver CameraImageSaverComp)

install(TARGETS ${target}
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
)
