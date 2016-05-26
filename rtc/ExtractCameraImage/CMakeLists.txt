set(comp_sources ExtractCameraImage.cpp)
set(libs hrpsysBaseStub)
add_library(ExtractCameraImage SHARED ${comp_sources})
target_link_libraries(ExtractCameraImage ${libs})
set_target_properties(ExtractCameraImage PROPERTIES PREFIX "")

add_executable(ExtractCameraImageComp ExtractCameraImageComp.cpp ${comp_sources})
target_link_libraries(ExtractCameraImageComp ${libs})

set(target ExtractCameraImage ExtractCameraImageComp)

install(TARGETS ${target}
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
)
