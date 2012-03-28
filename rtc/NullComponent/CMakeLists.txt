generate_stub_skel(NullService "OpenHRP")

set(comp_sources NullComponent.cpp NullServiceSk.cpp NullService_impl.cpp)
add_library(NullComponent SHARED ${comp_sources})
target_link_libraries(NullComponent ${OPENRTM_LIBRARIES})
set_target_properties(NullComponent PROPERTIES PREFIX "")

add_executable(NullComponentComp NullComponentComp.cpp ${comp_sources})
target_link_libraries(NullComponentComp ${OPENRTM_LIBRARIES})

set(target NullComponent NullComponentComp)

install(TARGETS ${target}
  RUNTIME DESTINATION bin CONFIGURATIONS Release Debug
  LIBRARY DESTINATION lib CONFIGURATIONS Release Debug
)
