set(comp_sources AutoBalancer.cpp AutoBalancerService_impl.cpp ../ImpedanceController/JointPathEx.cpp ../ImpedanceController/RatsMatrix.cpp ../SequencePlayer/interpolator.cpp PreviewController.cpp GaitGenerator.cpp)
set(libs hrpModel-3.1 hrpCollision-3.1 hrpUtil-3.1 hrpsysBaseStub)
add_library(AutoBalancer SHARED ${comp_sources})
target_link_libraries(AutoBalancer ${libs})
set_target_properties(AutoBalancer PROPERTIES PREFIX "")

add_executable(testPreviewController testPreviewController.cpp ../ImpedanceController/RatsMatrix.cpp PreviewController.cpp)
target_link_libraries(testPreviewController ${libs})

add_executable(testGaitGenerator testGaitGenerator.cpp ../ImpedanceController/RatsMatrix.cpp PreviewController.cpp GaitGenerator.cpp ../SequencePlayer/interpolator.cpp)
target_link_libraries(testGaitGenerator ${libs})

add_executable(AutoBalancerComp AutoBalancerComp.cpp ${comp_sources})
target_link_libraries(AutoBalancerComp ${libs})

include_directories(${PROJECT_SOURCE_DIR}/rtc/SequencePlayer)

set(target AutoBalancer AutoBalancerComp testPreviewController testGaitGenerator)

add_test(testPreviewControllerNoGP testPreviewController --use-gnuplot false)
add_test(testGaitGeneratorTest0 testGaitGenerator --test0 --use-gnuplot false)
add_test(testGaitGeneratorTest1 testGaitGenerator --test1 --use-gnuplot false)
add_test(testGaitGeneratorTest2 testGaitGenerator --test2 --use-gnuplot false)
add_test(testGaitGeneratorTest3 testGaitGenerator --test3 --use-gnuplot false)
add_test(testGaitGeneratorTest4 testGaitGenerator --test4 --use-gnuplot false)
add_test(testGaitGeneratorTest5 testGaitGenerator --test5 --use-gnuplot false)
#add_test(testGaitGeneratorTest6 testGaitGenerator --test6 --use-gnuplot false)
add_test(testGaitGeneratorTest7 testGaitGenerator --test7 --use-gnuplot false)
add_test(testGaitGeneratorTest8 testGaitGenerator --test8 --use-gnuplot false)
add_test(testGaitGeneratorTest9 testGaitGenerator --test9 --use-gnuplot false)
add_test(testGaitGeneratorTest10 testGaitGenerator --test10 --use-gnuplot false)
add_test(testGaitGeneratorTest11 testGaitGenerator --test11 --use-gnuplot false)
add_test(testGaitGeneratorTest12 testGaitGenerator --test12 --use-gnuplot false)
add_test(testGaitGeneratorTest13 testGaitGenerator --test13 --use-gnuplot false)
add_test(testGaitGeneratorTest14 testGaitGenerator --test14 --use-gnuplot false)
add_test(testGaitGeneratorTest15 testGaitGenerator --test15 --use-gnuplot false)
add_test(testGaitGeneratorTest16 testGaitGenerator --test16 --use-gnuplot false)

install(TARGETS ${target}
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
)

