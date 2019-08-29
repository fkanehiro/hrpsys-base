include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(comp_sources PlaneRemover.cpp)
set(libs hrpsysBaseStub ${PCL_LIBRARIES})
add_library(PlaneRemover SHARED ${comp_sources})
target_link_libraries(PlaneRemover ${libs})
set_target_properties(PlaneRemover PROPERTIES PREFIX "")

add_executable(PlaneRemoverComp PlaneRemoverComp.cpp ${comp_sources})
target_link_libraries(PlaneRemoverComp ${libs})

set(target PlaneRemover PlaneRemoverComp)

install(TARGETS ${target}
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
)
