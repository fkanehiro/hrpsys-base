set(comp_sources AverageFilter.cpp)
set(libs hrpsysBaseStub)
add_library(AverageFilter SHARED ${comp_sources})
target_link_libraries(AverageFilter ${libs})
set_target_properties(AverageFilter PROPERTIES PREFIX "")

add_executable(AverageFilterComp AverageFilterComp.cpp ${comp_sources})
target_link_libraries(AverageFilterComp ${libs})

set(target AverageFilter AverageFilterComp)

install(TARGETS ${target}
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
)
