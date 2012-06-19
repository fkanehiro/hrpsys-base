set(comp_sources VideoCapture.cpp camera.cpp)
set(libs ${OPENRTM_LIBRARIES} ${OpenCV_LIBRARIES} hrpsysBaseStub)
add_library(VideoCapture SHARED ${comp_sources})
target_link_libraries(VideoCapture ${libs})
set_target_properties(VideoCapture PROPERTIES PREFIX "")

add_executable(VideoCaptureComp VideoCaptureComp.cpp ${comp_sources})
target_link_libraries(VideoCaptureComp ${libs})

set(target VideoCapture VideoCaptureComp)

install(TARGETS ${target}
  RUNTIME DESTINATION bin CONFIGURATIONS Release Debug
  LIBRARY DESTINATION lib CONFIGURATIONS Release Debug
)
