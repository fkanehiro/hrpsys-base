option(BUILD_STABILIZER "Build Stabilizer RTC" ON)

if(NOT BUILD_STABILIZER)
  return()
endif()

set(comp_sources Integrator.cpp TwoDofController.cpp Stabilizer.cpp StabilizerService_impl.cpp ../ImpedanceController/JointPathEx.cpp ../ImpedanceController/RatsMatrix.cpp)
set(libs hrpModel-3.1 hrpUtil-3.1 hrpsysBaseStub)
add_library(Stabilizer SHARED ${comp_sources})
target_link_libraries(Stabilizer ${libs})
set_target_properties(Stabilizer PROPERTIES PREFIX "")
add_executable(StabilizerComp StabilizerComp.cpp ${comp_sources})
target_link_libraries(StabilizerComp ${libs})

set(target Stabilizer StabilizerComp)

add_executable(testTwoDofController testTwoDofController.cpp ${comp_sources})
target_link_libraries(testTwoDofController ${libs})
set(target Stabilizer StabilizerComp testTwoDofController)

install(TARGETS ${target}
  RUNTIME DESTINATION bin CONFIGURATIONS Release Debug
  LIBRARY DESTINATION lib CONFIGURATIONS Release Debug
)
