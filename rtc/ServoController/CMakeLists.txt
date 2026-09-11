set(comp_sources ServoController.cpp ServoControllerService_impl.cpp)
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

set(target ServoController ServoControllerComp)

install(TARGETS ${target}
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
)
