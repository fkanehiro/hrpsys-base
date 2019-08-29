set(comp_sources JpegDecoder.cpp)
set(libs ${OpenCV_LIBRARIES} hrpsysBaseStub)
add_library(JpegDecoder SHARED ${comp_sources})
target_link_libraries(JpegDecoder ${libs})
set_target_properties(JpegDecoder PROPERTIES PREFIX "")

add_executable(JpegDecoderComp JpegDecoderComp.cpp ${comp_sources})
target_link_libraries(JpegDecoderComp ${libs})

set(target JpegDecoder JpegDecoderComp)

install(TARGETS ${target}
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
)
