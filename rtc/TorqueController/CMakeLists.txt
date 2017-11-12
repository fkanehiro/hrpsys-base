set(comp_sources TorqueController.cpp ../Stabilizer/TwoDofController.cpp ../Stabilizer/Integrator.cpp MotorTorqueController.cpp TorqueControllerService_impl.cpp TwoDofControllerPDModel.cpp TwoDofControllerDynamicsModel.cpp Convolution.cpp)
set(libs hrpModel-${OPENHRP_LIBRARY_VERSION} hrpUtil-${OPENHRP_LIBRARY_VERSION} hrpsysBaseStub)
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
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
)
