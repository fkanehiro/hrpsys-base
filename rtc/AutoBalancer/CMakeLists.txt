set(comp_sources AutoBalancer.cpp AutoBalancerService_impl.cpp ../ImpedanceController/JointPathEx.cpp ../ImpedanceController/RatsMatrix.cpp ../SequencePlayer/interpolator.cpp PreviewController.cpp GaitGenerator.cpp)
set(libs hrpModel-3.1 hrpCollision-3.1 hrpUtil-3.1 hrpsysBaseStub)
add_library(AutoBalancer SHARED ${comp_sources})
target_link_libraries(AutoBalancer ${libs})
set_target_properties(AutoBalancer PROPERTIES PREFIX "")

add_executable(testPreviewController testPreviewController.cpp ../ImpedanceController/RatsMatrix.cpp PreviewController.cpp)
target_link_libraries(testPreviewController ${libs})

add_executable(testGaitGenerator testGaitGenerator.cpp ../ImpedanceController/RatsMatrix.cpp PreviewController.cpp GaitGenerator.cpp)
target_link_libraries(testGaitGenerator ${libs})

add_executable(AutoBalancerComp AutoBalancerComp.cpp ${comp_sources})
target_link_libraries(AutoBalancerComp ${libs})

include_directories(${PROJECT_SOURCE_DIR}/rtc/SequencePlayer)

set(target AutoBalancer AutoBalancerComp testPreviewController testGaitGenerator)

install(TARGETS ${target}
  RUNTIME DESTINATION bin CONFIGURATIONS Release Debug
  LIBRARY DESTINATION lib CONFIGURATIONS Release Debug
)

