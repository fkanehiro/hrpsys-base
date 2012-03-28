generate_stub_skel(HRPDataTypes "OpenHRP")

set(comp_sources Viewer.cpp IrrModel.cpp Project.cpp RTCGLbody.cpp HRPDataTypesSk.cpp HRPDataTypesDynSk.cpp)
include_directories(${LIBXML2_INCLUDE_DIR})
add_library(Viewer SHARED ${comp_sources})
target_link_libraries(Viewer ${OPENHRP_LIBRARIES} ${IRRLICHT_LIBRARIES} ${LIBXML2_LIBRARIES})
set_target_properties(Viewer PROPERTIES PREFIX "")

add_executable(ViewerComp ViewerComp.cpp ${comp_sources})
target_link_libraries(ViewerComp ${OPENHRP_LIBRARIES} ${IRRLICHT_LIBRARIES} ${LIBXML2_LIBRARIES})

set(target Viewer ViewerComp)

install(TARGETS ${target}
  RUNTIME DESTINATION bin CONFIGURATIONS Release Debug
  LIBRARY DESTINATION lib CONFIGURATIONS Release Debug
)
