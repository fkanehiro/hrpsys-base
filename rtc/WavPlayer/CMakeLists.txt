generate_stub_skel(WavPlayerService "OpenHRP")

set(comp_sources WavPlayer.cpp WavPlayerService_impl.cpp WavPlayerServiceSk.cpp) 
add_library(WavPlayer SHARED ${comp_sources})
target_link_libraries(WavPlayer ${OPENRTM_LIBRARIES})
set_target_properties(WavPlayer PROPERTIES PREFIX "")

add_executable(WavPlayerComp WavPlayerComp.cpp ${comp_sources})
target_link_libraries(WavPlayerComp ${OPENRTM_LIBRARIES})

set(target WavPlayer WavPlayerComp)

install(TARGETS ${target}
  RUNTIME DESTINATION bin CONFIGURATIONS Release Debug
  LIBRARY DESTINATION lib CONFIGURATIONS Release Debug
)

