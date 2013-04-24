set(comp_sources AutoBalancer.cpp AutoBalancerService_impl.cpp ../ImpedanceController/JointPathEx.cpp ../ImpedanceController/RatsMatrix.cpp PreviewController.cpp GaitGenerator.cpp)
set(libs hrpModel-3.1 hrpCollision-3.1 hrpUtil-3.1 hrpsysBaseStub)
add_library(AutoBalancer SHARED ${comp_sources})
target_link_libraries(AutoBalancer ${libs})
set_target_properties(AutoBalancer PROPERTIES PREFIX "")

#add_library(PreviewController SHARED PreviewController.cpp PreviewController.h)
#target_link_libraries(PreviewController ${libs})
#set_target_properties(PreviewController PROPERTIES PREFIX "")

#add_executable(TestPreviewController PreviewController.cpp PreviewController.h)
#set_target_properties(TestPreviewController PROPERTIES COMPILE_FLAGS "-DHAVE_MAIN")
#target_link_libraries(TestPreviewController ${libs})

#add_executable(TestGaitGenerator GaitGenerator.cpp GaitGenerator.h PreviewController.h)
#set_target_properties(TestGaitGenerator PROPERTIES COMPILE_FLAGS "-DHAVE_MAIN")
#target_link_libraries(TestGaitGenerator ${libs} PreviewController)

add_executable(AutoBalancerComp AutoBalancerComp.cpp ${comp_sources})
target_link_libraries(AutoBalancerComp ${libs})

#set(target AutoBalancer AutoBalancerComp TestPreviewController TestGaitGenerator)
set(target AutoBalancer AutoBalancerComp)

install(TARGETS ${target}
  RUNTIME DESTINATION bin CONFIGURATIONS Release Debug
  LIBRARY DESTINATION lib CONFIGURATIONS Release Debug
)

