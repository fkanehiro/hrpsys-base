option(BUILD_STABILIZER "Build Stabilizer RTC" ON)

if(NOT BUILD_STABILIZER)
  return()
endif()

if(USE_QPOASES)
  add_subdirectory(${CMAKE_CURRENT_BINARY_DIR}/../../3rdparty/qpOASES/qpOASES-3.0 ${CMAKE_CURRENT_BINARY_DIR}/../../3rdparty/qpOASES/qpOASES-3.0)
  link_directories(${CMAKE_CURRENT_BINARY_DIR}/../../3rdparty/qpOASES/qpOASES-3.0/bin)
  include_directories(${CMAKE_CURRENT_BINARY_DIR}/../../3rdparty/qpOASES/qpOASES-3.0/include)
  add_definitions(-DUSE_QPOASES)
endif()

set(comp_sources Integrator.cpp TwoDofController.cpp Stabilizer.cpp StabilizerService_impl.cpp ../ImpedanceController/JointPathEx.cpp ../ImpedanceController/RatsMatrix.cpp ../TorqueFilter/IIRFilter.h)
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

add_test(testZMPDistributorHRP2JSKTest0 testZMPDistributor --hrp2jsk --test0 --use-gnuplot false)
add_test(testZMPDistributorHRP2JSKTest1 testZMPDistributor --hrp2jsk --test1 --use-gnuplot false)
add_test(testZMPDistributorHRP2JSKTest2 testZMPDistributor --hrp2jsk --test2 --use-gnuplot false)
add_test(testZMPDistributorJAXONREDTest0 testZMPDistributor --jaxon_red --test0 --use-gnuplot false)
add_test(testZMPDistributorJAXONREDTest1 testZMPDistributor --jaxon_red --test1 --use-gnuplot false)
add_test(testZMPDistributorJAXONREDTest2 testZMPDistributor --jaxon_red --test2 --use-gnuplot false)

install(TARGETS ${target}
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
)
