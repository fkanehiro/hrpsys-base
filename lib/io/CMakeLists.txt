set(headers
  iob.h)

add_library(hrpIo SHARED iob.cpp)
if (NOT APPLE)
   target_link_libraries(hrpIo rt)
endif()

set(target hrpIo)

install(TARGETS ${target}
  RUNTIME DESTINATION bin CONFIGURATIONS Release Debug
  LIBRARY DESTINATION lib CONFIGURATIONS Release Debug
)

install(FILES ${headers} DESTINATION include/hrpsys/io)

