set(comp_sources PDcontroller.cpp)
add_library(PDcontroller SHARED ${comp_sources})
set(libs hrpModel-3.1 ${OPENRTM_LIBRARIES})
target_link_libraries(PDcontroller ${libs})
set_target_properties(PDcontroller PROPERTIES PREFIX "")

add_executable(PDcontrollerComp PDcontrollerComp.cpp ${comp_sources})
target_link_libraries(PDcontrollerComp ${libs})

set(target PDcontroller PDcontrollerComp)

install(TARGETS ${target}
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
)
