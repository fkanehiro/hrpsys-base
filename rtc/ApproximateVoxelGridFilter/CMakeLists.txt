include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(comp_sources ApproximateVoxelGridFilter.cpp)
set(libs hrpsysBaseStub ${PCL_LIBRARIES})
add_library(ApproximateVoxelGridFilter SHARED ${comp_sources})
target_link_libraries(ApproximateVoxelGridFilter ${libs})
set_target_properties(ApproximateVoxelGridFilter PROPERTIES PREFIX "")

add_executable(ApproximateVoxelGridFilterComp ApproximateVoxelGridFilterComp.cpp ${comp_sources})
target_link_libraries(ApproximateVoxelGridFilterComp ${libs})

set(target ApproximateVoxelGridFilter ApproximateVoxelGridFilterComp)

install(TARGETS ${target}
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
)
