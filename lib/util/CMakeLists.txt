set(sources
  Project.cpp
  ProjectUtil.cpp
  OpenRTMUtil.cpp
  ThreadedObject.cpp
  GLutil.cpp
  GLcoordinates.cpp
  GLcamera.cpp
  GLshape.cpp
  GLlink.cpp
  GLbody.cpp
  GLsceneBase.cpp
  GLbodyRTC.cpp
  SDLUtil.cpp
  BodyRTC.cpp
  BVutil.cpp
  PortHandler.cpp
  )

set(headers
  Project.h
  ProjectUtil.h
  OpenRTMUtil.h
  ThreadedObject.h
  GLutil.h
  GLcoordinates.h
  GLcamera.h
  GLshape.h
  GLlink.h
  GLbody.h
  GLsceneBase.h
  GLbodyRTC.h
  Hrpsys.h
  LogManagerBase.h
  LogManager.h
  SDLUtil.h
  VectorConvert.h
  BodyRTC.h
  BVutil.h
  PortHandler.h
  )

include_directories(${LIBXML2_INCLUDE_DIR})
include_directories(${SDL_INCLUDE_DIR})
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})

add_library(hrpsysUtil SHARED ${sources})

target_link_libraries(hrpsysUtil
  hrpsysBaseStub
  ${LIBXML2_LIBRARIES}
  ${OPENHRP_LIBRARIES}
  ${OPENGL_LIBRARIES} 
  ${OpenCV_LIBRARIES} 
  ${SDL_LIBRARY} 
  ${GLUT_LIBRARIES} 
  ${QHULL_LIBRARIES}
  GLEW
  boost_thread
  boost_system
  )

set(target hrpsysUtil)

install(TARGETS ${target}
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
)

install(FILES ${headers} DESTINATION include/hrpsys/util)
  
