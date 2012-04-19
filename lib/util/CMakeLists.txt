set(sources
  Project.cpp
  OpenRTMUtil.cpp
  ThreadedObject.cpp
  )

set(headers
  Project.h
  OpenRTMUtil.h
  semaphore.h
  ThreadedObject.h
  )

include_directories(${LIBXML2_INCLUDE_DIR})
include_directories(${SDL_INCLUDE_DIR})

add_library(hrpsysUtil SHARED ${sources})

target_link_libraries(hrpsysUtil
  ${LIBXML2_LIBRARIES}
  ${OPENHRP_LIBRARIES}
  ${SDL_LIBRARY} 
  )

set(target hrpsysUtil)

install(TARGETS ${target}
  RUNTIME DESTINATION bin CONFIGURATIONS Release Debug
  LIBRARY DESTINATION lib CONFIGURATIONS Release Debug
)

install(FILES ${headers} DESTINATION include/hrpsys/util)
  
