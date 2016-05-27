set(comp_sources ImageData2CameraImage.cpp)
set(libs hrpsysBaseStub)
add_library(ImageData2CameraImage SHARED ${comp_sources})
target_link_libraries(ImageData2CameraImage ${libs})
set_target_properties(ImageData2CameraImage PROPERTIES PREFIX "")

add_executable(ImageData2CameraImageComp ImageData2CameraImageComp.cpp ${comp_sources})
target_link_libraries(ImageData2CameraImageComp ${libs})

set(target ImageData2CameraImage ImageData2CameraImageComp)

install(TARGETS ${target}
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
)
