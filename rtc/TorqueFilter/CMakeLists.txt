set(comp_sources IIRFilter.cpp TorqueFilter.cpp)
set(libs hrpModel-3.1 hrpUtil-3.1 hrpsysBaseStub)
add_library(TorqueFilter SHARED ${comp_sources})
target_link_libraries(TorqueFilter ${libs})
set_target_properties(TorqueFilter PROPERTIES PREFIX "")

add_executable(TorqueFilterComp TorqueFilterComp.cpp ${comp_sources})
target_link_libraries(TorqueFilterComp ${libs})

set(target TorqueFilter TorqueFilterComp)

install(TARGETS ${target}
  RUNTIME DESTINATION bin CONFIGURATIONS Release Debug
  LIBRARY DESTINATION lib CONFIGURATIONS Release Debug
)
