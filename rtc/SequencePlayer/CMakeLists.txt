set(comp_sources interpolator.cpp timeUtil.cpp seqplay.cpp SequencePlayer.cpp SequencePlayerService_impl.cpp ../ImpedanceController/JointPathEx.cpp)
set(libs hrpModel-3.1 hrpCollision-3.1 hrpUtil-3.1 hrpsysBaseStub)
add_library(SequencePlayer SHARED ${comp_sources})
target_link_libraries(SequencePlayer ${libs})
set_target_properties(SequencePlayer PROPERTIES PREFIX "")

add_executable(SequencePlayerComp SequencePlayerComp.cpp ${comp_sources})
target_link_libraries(SequencePlayerComp ${libs})

set(target SequencePlayer SequencePlayerComp)

install(TARGETS ${target}
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
)

