include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${LIBXML2_INCLUDE_DIR})
set(comp_sources VirtualCamera.cpp GLscene.cpp RTCGLbody.cpp)

add_library(VirtualCamera SHARED ${comp_sources})
set(libraries
  hrpsysUtil
  hrpsysBaseStub
  )
target_link_libraries(VirtualCamera ${libraries})
set_target_properties(VirtualCamera PROPERTIES PREFIX "")

add_executable(VirtualCameraComp VirtualCameraComp.cpp ${comp_sources})
target_link_libraries(VirtualCameraComp ${libraries})

set(target VirtualCamera VirtualCameraComp)

install(TARGETS ${target}
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
)
