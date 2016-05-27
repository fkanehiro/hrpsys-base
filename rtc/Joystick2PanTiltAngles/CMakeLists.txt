add_library(Joystick2PanTiltAngles SHARED Joystick2PanTiltAngles.cpp)
target_link_libraries(Joystick2PanTiltAngles ${OPENRTM_LIBRARIES})
set_target_properties(Joystick2PanTiltAngles PROPERTIES PREFIX "")

add_executable(Joystick2PanTiltAnglesComp Joystick2PanTiltAnglesComp.cpp Joystick2PanTiltAngles.cpp)
target_link_libraries(Joystick2PanTiltAnglesComp ${OPENRTM_LIBRARIES})

set(target Joystick2PanTiltAngles Joystick2PanTiltAnglesComp)

install(TARGETS ${target}
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
)
