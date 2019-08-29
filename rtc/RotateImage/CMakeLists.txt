set(comp_sources RotateImage.cpp)
set(libs ${OpenCV_LIBRARIES} hrpsysBaseStub)
add_library(RotateImage SHARED ${comp_sources})
target_link_libraries(RotateImage ${libs})
set_target_properties(RotateImage PROPERTIES PREFIX "")

add_executable(RotateImageComp RotateImageComp.cpp ${comp_sources})
target_link_libraries(RotateImageComp ${libs})

set(target RotateImage RotateImageComp)

install(TARGETS ${target}
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
)
