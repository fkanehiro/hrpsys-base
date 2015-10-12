set(comp_sources ServoController.cpp ServoControllerService_impl.cpp)
set(libs hrpModel-3.1 hrpCollision-3.1 hrpUtil-3.1 hrpsysBaseStub)
if (CMAKE_COMPILER_IS_GNUCC)
    execute_process(COMMAND ${CMAKE_C_COMPILER} -dumpversion
                    OUTPUT_VARIABLE GCC_VERSION)
    string(REGEX MATCHALL "[0-9]+" GCC_VERSION_COMPONENTS
    ${GCC_VERSION})
    list(GET GCC_VERSION_COMPONENTS 0 GCC_MAJOR)
    list(GET GCC_VERSION_COMPONENTS 1 GCC_MINOR)
    if (GCC_VERSION VERSION_LESS 4.3)
       message(STATUS "GCC version (${GCC_MAJOR}.${GCC_MINOR}) must be at least 4.3 for binary format in ServoSerial.h!")
#    else()
#       message(STATUS "HOGE GCC version must be at least 4.3 for binary format in ServoSerial.h! ${GCC_MAJOR} ${GCC_MINOR}")
    endif()
    add_definitions(-DDONOT_USE_BINARY_FORMAT)
endif()

add_library(ServoController SHARED ${comp_sources})
target_link_libraries(ServoController ${libs})
set_target_properties(ServoController PROPERTIES PREFIX "")

add_executable(ServoControllerComp ServoControllerComp.cpp ${comp_sources})
target_link_libraries(ServoControllerComp ${libs})

add_executable(testServoSerial testServoSerial.cpp)

set(target ServoController ServoControllerComp)

install(TARGETS ${target}
  RUNTIME DESTINATION bin CONFIGURATIONS Release Debug
  LIBRARY DESTINATION lib CONFIGURATIONS Release Debug
)
