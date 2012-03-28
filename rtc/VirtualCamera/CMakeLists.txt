generate_stub_skel(Img "Img/")
generate_stub_skel(HRPDataTypes "OpenHRP")

find_package(OpenGL REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${LIBXML2_INCLUDE_DIR})
set(comp_sources VirtualCamera.cpp IrrModel.cpp Project.cpp ImgSk.cpp ImgDynSk.cpp HRPDataTypesSk.cpp HRPDataTypesDynSk.cpp RTCGLbody.cpp)

add_library(VirtualCamera SHARED ${comp_sources})
target_link_libraries(VirtualCamera ${OPENHRP_LIBRARIES} ${OPENGL_LIBRARIES} ${IRRLICHT_LIBRARIES} ${OpenCV_LIBRARIES} ${LIBXML2_LIBRARIES} rtcpcl_pointcloud_type)
set_target_properties(VirtualCamera PROPERTIES PREFIX "")

add_executable(VirtualCameraComp VirtualCameraComp.cpp ${comp_sources})
target_link_libraries(VirtualCameraComp ${OPENHRP_LIBRARIES} ${OPENGL_LIBRARIES} ${IRRLICHT_LIBRARIES} ${OpenCV_LIBRARIES} ${LIBXML2_LIBRARIES} rtcpcl_pointcloud_type)

set(target VirtualCamera VirtualCameraComp)

install(TARGETS ${target}
  RUNTIME DESTINATION bin CONFIGURATIONS Release Debug
  LIBRARY DESTINATION lib CONFIGURATIONS Release Debug
)
