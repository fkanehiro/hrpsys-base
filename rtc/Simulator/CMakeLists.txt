generate_stub_skel(HRPDataTypes "OpenHRP")

set(comp_sources Simulator.cpp Project.cpp RTCBody.cpp HRPDataTypesSk.cpp HRPDataTypesDynSk.cpp)
include_directories(${LIBXML2_INCLUDE_DIR})
add_library(Simulator SHARED ${comp_sources})
target_link_libraries(Simulator ${OPENHRP_LIBRARIES} ${LIBXML2_LIBRARIES})
set_target_properties(Simulator PROPERTIES PREFIX "")

add_executable(SimulatorComp SimulatorComp.cpp ${comp_sources})
target_link_libraries(SimulatorComp ${OPENHRP_LIBRARIES} ${LIBXML2_LIBRARIES})

set(target Simulator SimulatorComp)

install(TARGETS ${target}
  RUNTIME DESTINATION bin CONFIGURATIONS Release Debug
  LIBRARY DESTINATION lib CONFIGURATIONS Release Debug
)
