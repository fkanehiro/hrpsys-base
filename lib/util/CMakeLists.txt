set(sources
  Project.cpp
  ProjectUtil.cpp
  OpenRTMUtil.cpp
  ThreadedObject.cpp
  GLutil.cpp
  GLcamera.cpp
  GLlink.cpp
  GLbody.cpp
  GLsceneBase.cpp
  SDLUtil.cpp
  BodyRTC.cpp
  )

set(headers
  Project.h
  ProjectUtil.h
  OpenRTMUtil.h
  ThreadedObject.h
  GLutil.h
  GLcamera.h
  GLlink.h
  GLbody.h
  GLsceneBase.h
  LogManagerBase.h
  LogManager.h
  SDLUtil.h
  VectorConvert.h
  BodyRTC.h
  )

include_directories(${LIBXML2_INCLUDE_DIR})
include_directories(${SDL_INCLUDE_DIR})
include_directories(${OpenCV_INCLUDE_DIRS})

add_library(hrpsysUtil SHARED ${sources})

target_link_libraries(hrpsysUtil
  ${LIBXML2_LIBRARIES}
  ${OPENHRP_LIBRARIES}
  ${OPENGL_LIBRARIES} 
  ${OpenCV_LIBRARIES} 
  ${SDL_LIBRARY} 
  ${GLUT_LIBRARIES} 
  )

set(target hrpsysUtil)

install(TARGETS ${target}
  RUNTIME DESTINATION bin CONFIGURATIONS Release Debug
  LIBRARY DESTINATION lib CONFIGURATIONS Release Debug
)

install(FILES ${headers} DESTINATION include/hrpsys/util)
  
