add_library(hrpIo SHARED iob.cpp)

set(target hrpIo)

install(TARGETS ${target}
  RUNTIME DESTINATION bin CONFIGURATIONS Release Debug
  LIBRARY DESTINATION lib CONFIGURATIONS Release Debug
)
