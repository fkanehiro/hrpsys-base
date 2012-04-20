include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${LIBXML2_INCLUDE_DIR})
include_directories(${SDL_INCLUDE_DIR})

set(target hrpsys-monitor)

generate_stub_skel(RobotHardwareService "OpenHRP")
generate_stub_skel(StateHolderService "OpenHRP")

add_executable(hrpsys-monitor 
  GLmodel.cpp 
  SDLUtil.cpp
  Monitor.cpp
  main.cpp
  RobotHardwareServiceSk.cpp 
  StateHolderServiceSk.cpp 
  )

target_link_libraries(hrpsys-monitor 
  hrpsysUtil
  )

install(TARGETS ${target}
  RUNTIME DESTINATION bin
  )
