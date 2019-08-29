add_library(Joystick2Velocity3D SHARED Joystick2Velocity3D.cpp)
target_link_libraries(Joystick2Velocity3D ${OPENRTM_LIBRARIES})
set_target_properties(Joystick2Velocity3D PROPERTIES PREFIX "")

add_executable(Joystick2Velocity3DComp Joystick2Velocity3DComp.cpp Joystick2Velocity3D.cpp)
target_link_libraries(Joystick2Velocity3DComp ${OPENRTM_LIBRARIES})

set(target Joystick2Velocity3D Joystick2Velocity3DComp)

install(TARGETS ${target}
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
)
