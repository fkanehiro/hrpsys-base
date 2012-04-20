include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${LIBXML2_INCLUDE_DIR})
include_directories(${SDL_INCLUDE_DIR})

set(target hrpsys-viewer)

add_executable(hrpsys-viewer 
  GLscene.cpp 
  OnlineViewer_impl.cpp
  main.cpp
  )

target_link_libraries(hrpsys-viewer 
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
