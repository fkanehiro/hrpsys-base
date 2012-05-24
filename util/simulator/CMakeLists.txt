configure_file(hrpsys-simulator.in ${CMAKE_CURRENT_BINARY_DIR}/hrpsys-simulator)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${LIBXML2_INCLUDE_DIR})
include_directories(${SDL_INCLUDE_DIR})
find_package(PythonLibs)
include_directories(${PYTHON_INCLUDE_DIRS})

set(target sdlsim)

add_executable(sdlsim 
  GLscene.cpp 
  BodyState.cpp
  SceneState.cpp
  Simulator.cpp
  main.cpp
  )

target_link_libraries(sdlsim 
  hrpsysUtil
  )

add_library(hrpsys SHARED 
  GLscene.cpp 
  BodyState.cpp
  SceneState.cpp
  Simulator.cpp
  PySimulator.cpp
  PyBody.cpp
  PyLink.cpp
  )

target_link_libraries(hrpsys 
  boost_python 
  hrpsysUtil
  ${PYTHON_LIBRARIES}
  )
set_target_properties(hrpsys PROPERTIES PREFIX "")
set_target_properties(hrpsys PROPERTIES SUFFIX ".so")

install(TARGETS ${target}
  RUNTIME DESTINATION bin
  )

install(TARGETS hrpsys
  LIBRARY DESTINATION share/hrpsys/python CONFIGURATIONS Release Debug
)

install(PROGRAMS ${CMAKE_CURRENT_BINARY_DIR}/hrpsys-simulator 
  DESTINATION bin
  )  
