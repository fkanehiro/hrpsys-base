set(comp_sources Simulator.cpp RTCBody.cpp)
add_library(Simulator SHARED ${comp_sources})
set(libraries
  hrpsysUtil 
  hrpsysBaseStub
  )
target_link_libraries(Simulator ${libraries})
set_target_properties(Simulator PROPERTIES PREFIX "")

add_executable(SimulatorComp SimulatorComp.cpp ${comp_sources})
target_link_libraries(SimulatorComp ${libraries})

set(target Simulator SimulatorComp)

install(TARGETS ${target}
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
)
