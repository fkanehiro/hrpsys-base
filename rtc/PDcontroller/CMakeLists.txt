set(comp_sources PDcontroller.cpp)
add_library(PDcontroller SHARED ${comp_sources})
target_link_libraries(PDcontroller ${OPENRTM_LIBRARIES})
set_target_properties(PDcontroller PROPERTIES PREFIX "")

add_executable(PDcontrollerComp PDcontrollerComp.cpp ${comp_sources})
target_link_libraries(PDcontrollerComp ${OPENRTM_LIBRARIES})

set(target PDcontroller PDcontrollerComp)

install(TARGETS ${target}
  RUNTIME DESTINATION bin CONFIGURATIONS Release Debug
  LIBRARY DESTINATION lib CONFIGURATIONS Release Debug
)
