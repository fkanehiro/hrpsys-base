add_library(Joystick SHARED Joystick.cpp js.cpp)
target_link_libraries(Joystick ${OPENRTM_LIBRARIES})
set_target_properties(Joystick PROPERTIES PREFIX "")

add_executable(JoystickComp JoystickComp.cpp Joystick.cpp js.cpp)
target_link_libraries(JoystickComp ${OPENRTM_LIBRARIES})

set(target Joystick JoystickComp)

install(TARGETS ${target}
  RUNTIME DESTINATION bin CONFIGURATIONS Release Debug
  LIBRARY DESTINATION lib CONFIGURATIONS Release Debug
)
