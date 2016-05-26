include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(comp_sources SORFilter.cpp)
set(libs hrpsysBaseStub ${PCL_LIBRARIES})
add_library(SORFilter SHARED ${comp_sources})
target_link_libraries(SORFilter ${libs})
set_target_properties(SORFilter PROPERTIES PREFIX "")

add_executable(SORFilterComp SORFilterComp.cpp ${comp_sources})
target_link_libraries(SORFilterComp ${libs})

set(target SORFilter SORFilterComp)

install(TARGETS ${target}
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
)
