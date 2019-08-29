include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(comp_sources VoxelGridFilter.cpp)
set(libs hrpsysBaseStub ${PCL_LIBRARIES})
add_library(VoxelGridFilter SHARED ${comp_sources})
target_link_libraries(VoxelGridFilter ${libs})
set_target_properties(VoxelGridFilter PROPERTIES PREFIX "")

add_executable(VoxelGridFilterComp VoxelGridFilterComp.cpp ${comp_sources})
target_link_libraries(VoxelGridFilterComp ${libs})

set(target VoxelGridFilter VoxelGridFilterComp)

install(TARGETS ${target}
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
)
