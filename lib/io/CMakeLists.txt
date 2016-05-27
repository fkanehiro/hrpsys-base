set(headers
  iob.h)

add_library(hrpIo SHARED iob.cpp)
if (NOT APPLE AND NOT QNXNTO)
   target_link_libraries(hrpIo rt)
endif()

set(target hrpIo)

option(INSTALL_HRPIO "Install dummy implementation of hrpIo" ON)

if(INSTALL_HRPIO)
  install(TARGETS ${target}
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    )
endif()

install(FILES ${headers} DESTINATION include/hrpsys/io)

