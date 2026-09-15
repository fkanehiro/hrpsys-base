set(comp_sources ServoController.cpp ServoControllerService_impl.cpp)
option(SERVO_SERIAL_DEBUG "Log ServoController serial packets (may block callers)" OFF)
if(SERVO_SERIAL_DEBUG)
  add_definitions(-DSERVO_SERIAL_DEBUG)
endif()
find_package(Threads REQUIRED)
set(libs hrpModel-3.1 hrpCollision-3.1 hrpUtil-3.1 hrpsysBaseStub ${CMAKE_THREAD_LIBS_INIT})
add_library(ServoController SHARED ${comp_sources})
target_link_libraries(ServoController ${libs})
set_target_properties(ServoController PROPERTIES PREFIX "")

add_executable(ServoControllerComp ServoControllerComp.cpp ${comp_sources})
target_link_libraries(ServoControllerComp ${libs})

add_executable(testServoSerial testServoSerial.cpp)
if(CMAKE_THREAD_LIBS_INIT)
  target_link_libraries(testServoSerial ${CMAKE_THREAD_LIBS_INIT})
endif()

option(BUILD_SERVO_SERIAL_TEST "Build Linux PTY serial tests (no hardware access)" OFF)
if(BUILD_SERVO_SERIAL_TEST AND CMAKE_SYSTEM_NAME STREQUAL "Linux")
  add_executable(testServoSerialIO testServoSerialIO.cpp)
  target_link_libraries(testServoSerialIO ${CMAKE_THREAD_LIBS_INIT} util)
  foreach(cut RANGE 0 26)
    add_test(NAME testServoSerialIO-fragment-${cut} COMMAND testServoSerialIO fragment ${cut})
    set_tests_properties(testServoSerialIO-fragment-${cut} PROPERTIES TIMEOUT 3)
  endforeach()
  foreach(mode stale coalesced missing-echo missing-return checksum)
    add_test(NAME testServoSerialIO-${mode} COMMAND testServoSerialIO ${mode})
    set_tests_properties(testServoSerialIO-${mode} PROPERTIES TIMEOUT 3)
  endforeach()
endif()

set(target ServoController ServoControllerComp)

install(TARGETS ${target}
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
)
