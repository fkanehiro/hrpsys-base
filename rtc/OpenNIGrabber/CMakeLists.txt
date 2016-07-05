include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(comp_sources OpenNIGrabber.cpp)
set(libs hrpsysBaseStub ${PCL_LIBRARIES})
add_library(OpenNIGrabber SHARED ${comp_sources})
target_link_libraries(OpenNIGrabber ${libs})
set_target_properties(OpenNIGrabber PROPERTIES PREFIX "")

add_executable(OpenNIGrabberComp OpenNIGrabberComp.cpp ${comp_sources})
target_link_libraries(OpenNIGrabberComp ${libs})

set(target OpenNIGrabber OpenNIGrabberComp)

install(TARGETS ${target}
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
)
