generate_stub_skel(StateHolderService)
generate_stub_skel(TimeKeeperService)

set(comp_sources StateHolder.cpp StateHolderService_impl.cpp StateHolderServiceSk.cpp TimeKeeperService_impl.cpp TimeKeeperServiceSk.cpp)
add_library(StateHolder SHARED ${comp_sources})
target_link_libraries(StateHolder ${OPENRTM_LIBRARIES} hrpUtil-3.1)
set_target_properties(StateHolder PROPERTIES PREFIX "")

add_executable(StateHolderComp StateHolderComp.cpp ${comp_sources})
target_link_libraries(StateHolderComp ${OPENRTM_LIBRARIES} hrpUtil-3.1)

set(target StateHolder StateHolderComp)

install(TARGETS ${target}
  RUNTIME DESTINATION bin CONFIGURATIONS Release Debug
  LIBRARY DESTINATION lib CONFIGURATIONS Release Debug
)

