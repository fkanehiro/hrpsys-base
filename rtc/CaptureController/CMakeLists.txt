set(comp_sources CaptureController.cpp CameraCaptureService_impl.cpp)
set(libs ${OPENRTM_LIBRARIES} ${OpenCV_LIBRARIES} hrpsysBaseStub)
add_library(CaptureController SHARED ${comp_sources})
target_link_libraries(CaptureController ${libs})
set_target_properties(CaptureController PROPERTIES PREFIX "")

add_executable(CaptureControllerComp CaptureControllerComp.cpp ${comp_sources})
target_link_libraries(CaptureControllerComp ${libs})

set(target CaptureController CaptureControllerComp)

install(TARGETS ${target}
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
)
