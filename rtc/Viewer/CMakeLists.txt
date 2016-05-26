set(comp_sources Viewer.cpp GLscene.cpp RTCGLbody.cpp)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${LIBXML2_INCLUDE_DIR})
add_library(Viewer SHARED ${comp_sources})
set(libraries
  ${IRRLICHT_LIBRARIES}
  ${LIBXML2_LIBRARIES}  
  ${QHULL_LIBRARIES}
  ${OPENHRP_LIBRARIES}
  hrpsysUtil
  hrpsysBaseStub
  )
target_link_libraries(Viewer ${libraries})
set_target_properties(Viewer PROPERTIES PREFIX "")

add_executable(ViewerComp ViewerComp.cpp ${comp_sources})
target_link_libraries(ViewerComp ${libraries})

set(target Viewer ViewerComp)

install(TARGETS ${target}
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
)
