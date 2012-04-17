set(sources
  Project.cpp
  )

set(headers
  Project.h
  )

include_directories(${LIBXML2_INCLUDE_DIR})

add_library(hrpsysUtil SHARED ${sources})

target_link_libraries(hrpsysUtil
  ${LIBXML2_LIBRARIES}
  ${OPENHRP_LIBRARIES}
  )

set(target hrpsysUtil)

install(TARGETS ${target}
  RUNTIME DESTINATION bin CONFIGURATIONS Release Debug
  LIBRARY DESTINATION lib CONFIGURATIONS Release Debug
)

install(FILES ${headers} DESTINATION include/hrpsys/util)
  
