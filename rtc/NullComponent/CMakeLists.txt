set(comp_sources NullComponent.cpp NullService_impl.cpp)
set(libs hrpsysBaseStub)
add_library(NullComponent SHARED ${comp_sources})
target_link_libraries(NullComponent ${libs})
set_target_properties(NullComponent PROPERTIES PREFIX "")

add_executable(NullComponentComp NullComponentComp.cpp ${comp_sources})
target_link_libraries(NullComponentComp ${libs})

set(target NullComponent NullComponentComp)

install(TARGETS ${target}
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
)
