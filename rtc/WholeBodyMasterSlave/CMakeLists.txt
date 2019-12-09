set(comp_sources WholeBodyMasterSlave.cpp WholeBodyMasterSlaveService_impl.cpp ../ImpedanceController/JointPathEx.cpp ../ImpedanceController/RatsMatrix.cpp ../SequencePlayer/interpolator.cpp ../TorqueFilter/IIRFilter.cpp)
set(libs ${OpenCV_LIBRARIES} hrpModel-3.1 hrpCollision-3.1 hrpUtil-3.1 hrpsysBaseStub)
#set(CMAKE_CXX_STANDARD, 11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
add_library(WholeBodyMasterSlave SHARED ${comp_sources})
target_link_libraries(WholeBodyMasterSlave ${libs})
set_target_properties(WholeBodyMasterSlave PROPERTIES PREFIX "")

add_executable(WholeBodyMasterSlaveComp WholeBodyMasterSlaveComp.cpp ${comp_sources})
target_link_libraries(WholeBodyMasterSlaveComp ${libs})

include_directories(${PROJECT_SOURCE_DIR}/rtc/SequencePlayer)

set(target WholeBodyMasterSlave WholeBodyMasterSlaveComp )

#add_test(testPreviewControllerNoGP testPreviewController --use-gnuplot false)

install(TARGETS ${target}
  RUNTIME DESTINATION bin CONFIGURATIONS Release Debug
  LIBRARY DESTINATION lib CONFIGURATIONS Release Debug
)

