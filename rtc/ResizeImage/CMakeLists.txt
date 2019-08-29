set(comp_sources ResizeImage.cpp)
set(libs ${OpenCV_LIBRARIES} hrpsysBaseStub)
add_library(ResizeImage SHARED ${comp_sources})
target_link_libraries(ResizeImage ${libs})
set_target_properties(ResizeImage PROPERTIES PREFIX "")

add_executable(ResizeImageComp ResizeImageComp.cpp ${comp_sources})
target_link_libraries(ResizeImageComp ${libs})

set(target ResizeImage ResizeImageComp)

install(TARGETS ${target}
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
)
