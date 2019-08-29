set(comp_sources ObjectContactTurnaroundDetector.cpp ObjectContactTurnaroundDetectorService_impl.cpp ObjectContactTurnaroundDetectorBase.h ../ImpedanceController/RatsMatrix.cpp ../TorqueFilter/IIRFilter.cpp)
set(libs hrpModel-3.1 hrpCollision-3.1 hrpUtil-3.1 hrpsysBaseStub)
add_library(ObjectContactTurnaroundDetector SHARED ${comp_sources})
target_link_libraries(ObjectContactTurnaroundDetector ${libs})
set_target_properties(ObjectContactTurnaroundDetector PROPERTIES PREFIX "")

add_executable(ObjectContactTurnaroundDetectorComp ObjectContactTurnaroundDetectorComp.cpp ${comp_sources})
target_link_libraries(ObjectContactTurnaroundDetectorComp ${libs})
add_executable(testObjectContactTurnaroundDetectorBase testObjectContactTurnaroundDetectorBase.cpp ObjectContactTurnaroundDetectorBase.h ../TorqueFilter/IIRFilter.cpp)
target_link_libraries(testObjectContactTurnaroundDetectorBase ${libs})

set(target ObjectContactTurnaroundDetector ObjectContactTurnaroundDetectorComp testObjectContactTurnaroundDetectorBase)

add_test(testObjectContactTurnaroundDetectorBase0 testObjectContactTurnaroundDetectorBase --test0 --use-gnuplot false)
add_test(testObjectContactTurnaroundDetectorBase1 testObjectContactTurnaroundDetectorBase --test1 --use-gnuplot false)
add_test(testObjectContactTurnaroundDetectorBase2 testObjectContactTurnaroundDetectorBase --test2 --use-gnuplot false)
add_test(testObjectContactTurnaroundDetectorBase3 testObjectContactTurnaroundDetectorBase --test3 --use-gnuplot false)
add_test(testObjectContactTurnaroundDetectorBase4 testObjectContactTurnaroundDetectorBase --test4 --use-gnuplot false)
add_test(testObjectContactTurnaroundDetectorBase5 testObjectContactTurnaroundDetectorBase --test5 --use-gnuplot false)
add_test(testObjectContactTurnaroundDetectorBase6 testObjectContactTurnaroundDetectorBase --test6 --use-gnuplot false)
add_test(testObjectContactTurnaroundDetectorBase7 testObjectContactTurnaroundDetectorBase --test7 --use-gnuplot false)

install(TARGETS ${target}
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
)

