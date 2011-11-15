generate_stub_skel_pkg(Img "Img/")

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

set(comp_sources VideoCapture.cpp ImgSk.cpp ImgDynSk.cpp camera.cpp)
add_library(VideoCapture SHARED ${comp_sources})
target_link_libraries(VideoCapture ${OPENRTM_LIBRARIES} ${OpenCV_LIBRARIES})
set_target_properties(VideoCapture PROPERTIES PREFIX "")

add_executable(VideoCaptureComp VideoCaptureComp.cpp ${comp_sources})
target_link_libraries(VideoCaptureComp ${OPENRTM_LIBRARIES} ${OpenCV_LIBRARIES})

set(target VideoCapture VideoCaptureComp)

install(TARGETS ${target}
  RUNTIME DESTINATION bin CONFIGURATIONS Release Debug
  LIBRARY DESTINATION lib CONFIGURATIONS Release Debug
)
