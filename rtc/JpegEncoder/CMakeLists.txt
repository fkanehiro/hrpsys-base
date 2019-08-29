set(comp_sources JpegEncoder.cpp)
set(libs ${OpenCV_LIBRARIES} hrpsysBaseStub)
add_library(JpegEncoder SHARED ${comp_sources})
target_link_libraries(JpegEncoder ${libs})
set_target_properties(JpegEncoder PROPERTIES PREFIX "")

add_executable(JpegEncoderComp JpegEncoderComp.cpp ${comp_sources})
target_link_libraries(JpegEncoderComp ${libs})

set(target JpegEncoder JpegEncoderComp)

install(TARGETS ${target}
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
)
