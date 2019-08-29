add_library(Joystick2Velocity2D SHARED Joystick2Velocity2D.cpp)
target_link_libraries(Joystick2Velocity2D ${OPENRTM_LIBRARIES})
set_target_properties(Joystick2Velocity2D PROPERTIES PREFIX "")

add_executable(Joystick2Velocity2DComp Joystick2Velocity2DComp.cpp Joystick2Velocity2D.cpp)
target_link_libraries(Joystick2Velocity2DComp ${OPENRTM_LIBRARIES})

set(target Joystick2Velocity2D Joystick2Velocity2DComp)

install(TARGETS ${target}
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
)
