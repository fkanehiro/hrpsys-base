generate_stub_skel(Img "Img")

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

set(comp_sources JpegDecoder.cpp ImgSk.cpp ImgDynSk.cpp)
add_library(JpegDecoder SHARED ${comp_sources})
target_link_libraries(JpegDecoder ${OPENRTM_LIBRARIES} ${OpenCV_LIBRARIES})
set_target_properties(JpegDecoder PROPERTIES PREFIX "")

add_executable(JpegDecoderComp JpegDecoderComp.cpp ${comp_sources})
target_link_libraries(JpegDecoderComp ${OPENRTM_LIBRARIES} ${OpenCV_LIBRARIES})

set(target JpegDecoder JpegDecoderComp)

install(TARGETS ${target}
  RUNTIME DESTINATION bin CONFIGURATIONS Release Debug
  LIBRARY DESTINATION lib CONFIGURATIONS Release Debug
)
