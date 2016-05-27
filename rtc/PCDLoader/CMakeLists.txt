include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(comp_sources PCDLoader.cpp)
set(libs hrpsysBaseStub ${PCL_LIBRARIES})
add_library(PCDLoader SHARED ${comp_sources})
target_link_libraries(PCDLoader ${libs})
set_target_properties(PCDLoader PROPERTIES PREFIX "")

add_executable(PCDLoaderComp PCDLoaderComp.cpp ${comp_sources})
target_link_libraries(PCDLoaderComp ${libs})

set(target PCDLoader PCDLoaderComp)

install(TARGETS ${target}
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
)
