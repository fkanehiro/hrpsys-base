if (APPLE)
   set(js_src js_mac.cpp ysjoyreader.cpp ysjoyreader-objc.m)
   set(js_lib "-framework IOKit -framework Foundation")
else()
   set(js_src js.cpp)
   set(js_lib)
endif()

add_library(Joystick SHARED Joystick.cpp ${js_src})
target_link_libraries(Joystick ${OPENRTM_LIBRARIES} ${js_lib})
set_target_properties(Joystick PROPERTIES PREFIX "")

add_executable(JoystickComp JoystickComp.cpp Joystick.cpp ${js_src})
target_link_libraries(JoystickComp ${OPENRTM_LIBRARIES} ${js_lib})

set(target Joystick JoystickComp)

install(TARGETS ${target}
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
)
