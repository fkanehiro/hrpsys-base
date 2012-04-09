configure_file(hrpsys-simulator.in ${CMAKE_CURRENT_BINARY_DIR}/hrpsys-simulator)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${LIBXML2_INCLUDE_DIR})
include_directories(${SDL_INCLUDE_DIR})

set(target sdlsim)

add_executable(sdlsim 
  GLmodel.cpp 
  BodyRTC.cpp
  OpenRTMUtil.cpp
  Project.cpp
  ProjectUtil.cpp
  SDLUtil.cpp
  Simulator.cpp
  sdlsim.cpp
  )

target_link_libraries(sdlsim 
  ${OPENHRP_LIBRARIES} 
  ${OpenCV_LIBRARIES} 
  ${OPENGL_LIBRARIES} 
  ${GLUT_LIBRARIES} 
  ${SDL_LIBRARY} 
  ${LIBXML2_LIBRARIES})

install(TARGETS ${target}
  RUNTIME DESTINATION bin
  )
install(PROGRAMS ${CMAKE_CURRENT_BINARY_DIR}/hrpsys-simulator 
  DESTINATION bin
  )  
