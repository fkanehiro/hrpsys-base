set(comp_sources WavPlayer.cpp WavPlayerService_impl.cpp) 
set(libs hrpsysBaseStub)
add_library(WavPlayer SHARED ${comp_sources})
target_link_libraries(WavPlayer ${libs})
set_target_properties(WavPlayer PROPERTIES PREFIX "")

add_executable(WavPlayerComp WavPlayerComp.cpp ${comp_sources})
target_link_libraries(WavPlayerComp ${libs})

set(target WavPlayer WavPlayerComp)

install(TARGETS ${target}
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
)

