# set(comp_sources TorqueControllerService_impl.cpp TorqueController.cpp ../Stabilizer/TwoDofController.cpp)
set(comp_sources TorqueController.cpp ../Stabilizer/TwoDofController.cpp MotorTorqueController.cpp TorqueControllerService_impl.cpp)
set(libs hrpModel-3.1 hrpUtil-3.1 hrpsysBaseStub)
add_library(TorqueController SHARED ${comp_sources})
target_link_libraries(TorqueController ${libs})
set_target_properties(TorqueController PROPERTIES PREFIX "")

add_executable(TorqueControllerComp TorqueControllerComp.cpp ${comp_sources})
target_link_libraries(TorqueControllerComp ${libs})

add_executable(testMotorTorqueController testMotorTorqueController.cpp ${comp_sources})
target_link_libraries(testMotorTorqueController ${libs})

# set(target TorqueController TorqueControllerComp)
set(target TorqueController TorqueControllerComp testMotorTorqueController)

install(TARGETS ${target}
  RUNTIME DESTINATION bin CONFIGURATIONS Release Debug
  LIBRARY DESTINATION lib CONFIGURATIONS Release Debug
)
