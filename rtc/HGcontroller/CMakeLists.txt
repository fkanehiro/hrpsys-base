set(comp_sources HGcontroller.cpp)
add_library(HGcontroller SHARED ${comp_sources})
target_link_libraries(HGcontroller ${OPENRTM_LIBRARIES})
set_target_properties(HGcontroller PROPERTIES PREFIX "")

add_executable(HGcontrollerComp HGcontrollerComp.cpp ${comp_sources})
target_link_libraries(HGcontrollerComp ${OPENRTM_LIBRARIES})

set(target HGcontroller HGcontrollerComp)

install(TARGETS ${target}
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
)
