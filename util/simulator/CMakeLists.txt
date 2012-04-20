configure_file(hrpsys-simulator.in ${CMAKE_CURRENT_BINARY_DIR}/hrpsys-simulator)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${LIBXML2_INCLUDE_DIR})
include_directories(${SDL_INCLUDE_DIR})

set(target sdlsim)

add_executable(sdlsim 
  GLscene.cpp 
  BodyRTC.cpp
  BodyState.cpp
  SceneState.cpp
  ProjectUtil.cpp
  Simulator.cpp
  main.cpp
  )

target_link_libraries(sdlsim 
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
install(PROGRAMS ${CMAKE_CURRENT_BINARY_DIR}/hrpsys-simulator 
  DESTINATION bin
  )  
