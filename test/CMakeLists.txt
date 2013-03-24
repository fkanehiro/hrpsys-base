# check if gtest exists
if (EXISTS "/usr/src/gtest")

  # setup libraries
  set(comp_source  robot.cpp RobotHardware.cpp RobotHardwareService_impl.cpp)
  set(libs hrpIo hrpModel-3.1 hrpCollision-3.1 hrpUtil-3.1 hrpsysBaseStub)
  link_directories(${LIBIO_DIR})

  # compile gtest
  add_subdirectory("/usr/src/gtest/" ${CMAKE_BINARY_DIR}/_gtest_from_src)

  # add includes
  include_directories(${PROJECT_SOURCE_DIR})

  # test for RobotHardware
  add_executable(test_RobotHardware test_RobotHardware.cpp ${PROJECT_SOURCE_DIR}/rtc/RobotHardware/robot.cpp)
  add_dependencies(test_RobotHardware gtest)
  target_link_libraries(test_RobotHardware ${libs} gtest)

  # test for SequencePlayer
  add_executable(test_SequencePlayer test_SequencePlayer.cpp ${PROJECT_SOURCE_DIR}/rtc/SequencePlayer/seqplay.cpp ${PROJECT_SOURCE_DIR}/rtc/SequencePlayer/interpolator.cpp ${PROJECT_SOURCE_DIR}/rtc/SequencePlayer/timeUtil.cpp)
  add_dependencies(test_SequencePlayer gtest)
  target_link_libraries(test_SequencePlayer ${libs} gtest)

  #
  add_definitions(-DPROJECT_SOURCE_DIR=\\"${PROJECT_SOURCE_DIR}\\")

else (EXISTS "/usr/src/gtest")
  message(WARNING "GTest not found; C++ tests will fail to build.")
endif (EXISTS "/usr/src/gtest")

