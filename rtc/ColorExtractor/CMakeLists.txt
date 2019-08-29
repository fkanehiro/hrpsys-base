set(comp_sources ColorExtractor.cpp)
set(libs ${OpenCV_LIBRARIES} hrpsysBaseStub)
add_library(ColorExtractor SHARED ${comp_sources})
target_link_libraries(ColorExtractor ${libs})
set_target_properties(ColorExtractor PROPERTIES PREFIX "")

add_executable(ColorExtractorComp ColorExtractorComp.cpp ${comp_sources})
target_link_libraries(ColorExtractorComp ${libs})

set(target ColorExtractor ColorExtractorComp)

install(TARGETS ${target}
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
)
