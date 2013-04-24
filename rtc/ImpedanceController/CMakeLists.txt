set(comp_sources ImpedanceController.cpp ImpedanceControllerService_impl.cpp JointPathEx.cpp RatsMatrix.cpp)
set(libs hrpModel-3.1 hrpCollision-3.1 hrpUtil-3.1 hrpsysBaseStub)
add_library(ImpedanceController SHARED ${comp_sources})
target_link_libraries(ImpedanceController ${libs})
set_target_properties(ImpedanceController PROPERTIES PREFIX "")

add_executable(ImpedanceControllerComp ImpedanceControllerComp.cpp ${comp_sources})
target_link_libraries(ImpedanceControllerComp ${libs})

set(target ImpedanceController ImpedanceControllerComp)

install(TARGETS ${target}
  RUNTIME DESTINATION bin CONFIGURATIONS Release Debug
  LIBRARY DESTINATION lib CONFIGURATIONS Release Debug
)

