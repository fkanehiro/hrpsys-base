configure_file(hrpsys-simulator.in ${CMAKE_CURRENT_BINARY_DIR}/hrpsys-simulator)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${LIBXML2_INCLUDE_DIR})
include_directories(${SDL_INCLUDE_DIR})
include_directories(/usr/include/python2.6)

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
  hrpsysUtil
  )

add_library(simulator SHARED 
  GLscene.cpp 
  BodyRTC.cpp
  BodyState.cpp
  SceneState.cpp
  ProjectUtil.cpp
  Simulator.cpp
  PySimulator.cpp)

target_link_libraries(simulator 
  boost_python 
  hrpsysUtil
  )
set_target_properties(simulator PROPERTIES PREFIX "")

install(TARGETS ${target}
  RUNTIME DESTINATION bin
  )

install(TARGETS simulator
  LIBRARY DESTINATION share/hrpsys/python CONFIGURATIONS Release Debug
)

install(PROGRAMS ${CMAKE_CURRENT_BINARY_DIR}/hrpsys-simulator 
  DESTINATION bin
  )  
