set(comp_sources StateHolder.cpp StateHolderService_impl.cpp TimeKeeperService_impl.cpp)
set(libs hrpUtil-${OPENHRP_LIBRARY_VERSION} hrpModel-${OPENHRP_LIBRARY_VERSION} hrpsysBaseStub)
add_library(StateHolder SHARED ${comp_sources})
target_link_libraries(StateHolder ${libs})
set_target_properties(StateHolder PROPERTIES PREFIX "")

add_executable(StateHolderComp StateHolderComp.cpp ${comp_sources})
target_link_libraries(StateHolderComp ${libs})

set(target StateHolder StateHolderComp)

install(TARGETS ${target}
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
)

