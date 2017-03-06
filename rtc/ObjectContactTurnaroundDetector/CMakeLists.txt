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

install(TARGETS ${target}
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
)

