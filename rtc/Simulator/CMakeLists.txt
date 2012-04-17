generate_stub_skel(HRPDataTypes "OpenHRP")

set(comp_sources Simulator.cpp RTCBody.cpp HRPDataTypesSk.cpp HRPDataTypesDynSk.cpp)
include_directories(${LIBXML2_INCLUDE_DIR})
add_library(Simulator SHARED ${comp_sources})
set(libraries
  ${OPENHRP_LIBRARIES} 
  hrpsysUtil 
  ${LIBXML2_LIBRARIES}
  )
target_link_libraries(Simulator ${libraries})
set_target_properties(Simulator PROPERTIES PREFIX "")

add_executable(SimulatorComp SimulatorComp.cpp ${comp_sources})
target_link_libraries(SimulatorComp ${libraries})

set(target Simulator SimulatorComp)

install(TARGETS ${target}
  RUNTIME DESTINATION bin CONFIGURATIONS Release Debug
  LIBRARY DESTINATION lib CONFIGURATIONS Release Debug
)
