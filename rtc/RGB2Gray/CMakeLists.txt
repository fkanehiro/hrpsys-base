set(comp_sources RGB2Gray.cpp)
set(libs ${OpenCV_LIBRARIES} hrpsysBaseStub)
add_library(RGB2Gray SHARED ${comp_sources})
target_link_libraries(RGB2Gray ${libs})
set_target_properties(RGB2Gray PROPERTIES PREFIX "")

add_executable(RGB2GrayComp RGB2GrayComp.cpp ${comp_sources})
target_link_libraries(RGB2GrayComp ${libs})

set(target RGB2Gray RGB2GrayComp)

install(TARGETS ${target}
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
)
