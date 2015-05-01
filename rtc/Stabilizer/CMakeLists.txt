option(BUILD_STABILIZER "Build Stabilizer RTC" ON)

if(NOT BUILD_STABILIZER)
  return()
endif()

option(USE_QPOASES "Build qpOASES" OFF)
set(USE_QPOASES OFF)
message(STATUS "USE_QPOASES=${USE_QPOASES}")
if(USE_QPOASES)
  add_subdirectory(qpOASES-source)
  link_directories(qpOASES-source/libs)
  include_directories(qpOASES-source/include)
  add_definitions(-DUSE_QPOASES)
endif()

set(comp_sources Integrator.cpp TwoDofController.cpp Stabilizer.cpp StabilizerService_impl.cpp ../ImpedanceController/JointPathEx.cpp ../ImpedanceController/RatsMatrix.cpp)
if(USE_QPOASES)
  set(libs hrpModel-3.1 hrpUtil-3.1 hrpsysBaseStub qpOASES)
else()
  set(libs hrpModel-3.1 hrpUtil-3.1 hrpsysBaseStub)
endif()
add_library(Stabilizer SHARED ${comp_sources})
target_link_libraries(Stabilizer ${libs})
set_target_properties(Stabilizer PROPERTIES PREFIX "")
add_executable(StabilizerComp StabilizerComp.cpp ${comp_sources})
target_link_libraries(StabilizerComp ${libs})

set(target Stabilizer StabilizerComp)

add_executable(testTwoDofController testTwoDofController.cpp ${comp_sources})
target_link_libraries(testTwoDofController ${libs})
add_executable(testZMPDistributor testZMPDistributor.cpp ZMPDistributor.h ../ImpedanceController/JointPathEx.cpp)
target_link_libraries(testZMPDistributor ${libs})
set(target Stabilizer StabilizerComp testTwoDofController testZMPDistributor)


install(TARGETS ${target}
  RUNTIME DESTINATION bin CONFIGURATIONS Release Debug
  LIBRARY DESTINATION lib CONFIGURATIONS Release Debug
)
