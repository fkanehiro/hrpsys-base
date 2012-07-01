set(comp_sources DataLogger.cpp DataLoggerService_impl.cpp)
set(libs hrpsysBaseStub)
add_library(DataLogger SHARED ${comp_sources})
target_link_libraries(DataLogger ${libs})
set_target_properties(DataLogger PROPERTIES PREFIX "")

add_executable(DataLoggerComp DataLoggerComp.cpp ${comp_sources})
target_link_libraries(DataLoggerComp ${libs})

set(target DataLogger DataLoggerComp)

install(TARGETS ${target}
  RUNTIME DESTINATION bin CONFIGURATIONS Release Debug
  LIBRARY DESTINATION lib CONFIGURATIONS Release Debug
)

