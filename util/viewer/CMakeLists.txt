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
  hrpsysUtil
  )

install(TARGETS ${target}
  RUNTIME DESTINATION bin
  )
