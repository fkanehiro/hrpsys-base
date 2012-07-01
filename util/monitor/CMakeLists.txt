include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${LIBXML2_INCLUDE_DIR})
include_directories(${SDL_INCLUDE_DIR})

set(target hrpsys-monitor)

add_executable(hrpsys-monitor 
  GLscene.cpp 
  Monitor.cpp
  main.cpp
  )

target_link_libraries(hrpsys-monitor 
  hrpsysUtil
  hrpsysBaseStub
  )

install(TARGETS ${target}
  RUNTIME DESTINATION bin
  )
