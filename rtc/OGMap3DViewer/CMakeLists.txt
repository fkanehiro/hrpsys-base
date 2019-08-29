set(comp_sources OGMap3DViewer.cpp IrrModel.cpp)
set(libs ${OPENHRP_LIBRARIES} ${OPENGL_LIBRARIES} ${IRRLICHT_LIBRARIES} ${OpenCV_LIBRARIES} hrpsysBaseStub)
add_library(OGMap3DViewer SHARED ${comp_sources})
target_link_libraries(OGMap3DViewer ${libs})
set_target_properties(OGMap3DViewer PROPERTIES PREFIX "")

add_executable(OGMap3DViewerComp OGMap3DViewerComp.cpp ${comp_sources})
target_link_libraries(OGMap3DViewerComp ${libs})

set(target OGMap3DViewer OGMap3DViewerComp)

install(TARGETS ${target}
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
)
