include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${LIBXML2_INCLUDE_DIR})
include_directories(${SDL_INCLUDE_DIR})

set(target hrpsys-monitor)

generate_stub_skel(RobotHardwareService "OpenHRP")

add_executable(hrpsys-monitor 
  GLmodel.cpp 
  BodyState.cpp
  SceneState.cpp
  OpenRTMUtil.cpp
  ProjectUtil.cpp
  SDLUtil.cpp
  main.cpp
  RobotHardwareServiceSk.cpp 
  )

target_link_libraries(hrpsys-monitor 
  ${OPENHRP_LIBRARIES} 
  hrpsysUtil
  ${OpenCV_LIBRARIES} 
  ${OPENGL_LIBRARIES} 
  ${GLUT_LIBRARIES} 
  ${SDL_LIBRARY} 
  )

install(TARGETS ${target}
  RUNTIME DESTINATION bin
  )
