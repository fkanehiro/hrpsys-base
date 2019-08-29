include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(comp_sources MLSFilter.cpp)
set(libs hrpsysBaseStub ${PCL_LIBRARIES})
add_library(MLSFilter SHARED ${comp_sources})
target_link_libraries(MLSFilter ${libs})
set_target_properties(MLSFilter PROPERTIES PREFIX "")

add_executable(MLSFilterComp MLSFilterComp.cpp ${comp_sources})
target_link_libraries(MLSFilterComp ${libs})

set(target MLSFilter MLSFilterComp)

install(TARGETS ${target}
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
)
