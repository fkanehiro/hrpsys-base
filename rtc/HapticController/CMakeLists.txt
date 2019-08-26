set(comp_sources HapticController.cpp HapticControllerService_impl.cpp ../ImpedanceController/JointPathEx.cpp ../ImpedanceController/RatsMatrix.cpp ../SequencePlayer/interpolator.cpp ../TorqueFilter/IIRFilter.cpp)
set(libs ${OpenCV_LIBRARIES} hrpModel-3.1 hrpCollision-3.1 hrpUtil-3.1 hrpsysBaseStub)
add_library(HapticController SHARED ${comp_sources})
target_link_libraries(HapticController ${libs})
set_target_properties(HapticController PROPERTIES PREFIX "")

add_executable(HapticControllerComp HapticControllerComp.cpp ${comp_sources})
target_link_libraries(HapticControllerComp ${libs})

include_directories(${PROJECT_SOURCE_DIR}/rtc/SequencePlayer)

set(target HapticController HapticControllerComp )

#add_test(testPreviewControllerNoGP testPreviewController --use-gnuplot false)

install(TARGETS ${target}
  RUNTIME DESTINATION bin CONFIGURATIONS Release Debug
  LIBRARY DESTINATION lib CONFIGURATIONS Release Debug
)

