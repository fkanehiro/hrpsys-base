set(comp_sources UndistortImage.cpp)
set(libs hrpsysBaseStub ${OpenCV_LIBRARIES})
add_library(UndistortImage SHARED ${comp_sources})
target_link_libraries(UndistortImage ${libs})
set_target_properties(UndistortImage PROPERTIES PREFIX "")

add_executable(UndistortImageComp UndistortImageComp.cpp ${comp_sources})
target_link_libraries(UndistortImageComp ${libs})

set(target UndistortImage UndistortImageComp)

install(TARGETS ${target}
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
)
