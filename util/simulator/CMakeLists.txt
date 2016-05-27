configure_file(hrpsys-simulator-jython.in ${CMAKE_CURRENT_BINARY_DIR}/hrpsys-simulator-jython)
configure_file(hrpsys-simulator-python.in ${CMAKE_CURRENT_BINARY_DIR}/hrpsys-simulator-python)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${LIBXML2_INCLUDE_DIR})
include_directories(${SDL_INCLUDE_DIR})
find_package(PythonLibs)
include_directories(${PYTHON_INCLUDE_DIRS})

set(target hrpsys-simulator)

add_executable(hrpsys-simulator 
  GLscene.cpp 
  BodyState.cpp
  SceneState.cpp
  Simulator.cpp
  main.cpp
  )

target_link_libraries(hrpsys-simulator 
  hrpsysUtil
  )

add_library(hrpsysext SHARED 
  GLscene.cpp 
  BodyState.cpp
  SceneState.cpp
  Simulator.cpp
  PySimulator.cpp
  PyBody.cpp
  PyLink.cpp
  PyShape.cpp
  )

if (NOT APPLE)
   target_link_libraries(hrpsysext
     boost_python
     hrpsysUtil
     ${PYTHON_LIBRARIES}
     )
else()
   target_link_libraries(hrpsysext 
     boost_python
     hrpsysUtil
     ${PYTHON_LIBRARIES}
     )
endif()

set_target_properties(hrpsysext PROPERTIES PREFIX "")
set_target_properties(hrpsysext PROPERTIES SUFFIX ".so")

install(TARGETS ${target}
  RUNTIME DESTINATION bin
  )

install(TARGETS hrpsysext
  LIBRARY DESTINATION ${python_dist_pkg_dir}/hrpsys
)

install(PROGRAMS 
  ${CMAKE_CURRENT_BINARY_DIR}/hrpsys-simulator-jython
  ${CMAKE_CURRENT_BINARY_DIR}/hrpsys-simulator-python
  DESTINATION bin
  )  
