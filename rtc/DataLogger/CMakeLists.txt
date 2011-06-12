generate_stub_skel(DataLoggerService)

set(comp_sources DataLogger.cpp DataLoggerService_impl.cpp DataLoggerServiceSk.cpp )
add_library(DataLogger SHARED ${comp_sources})
target_link_libraries(DataLogger ${OPENRTM_LIBRARIES})
set_target_properties(DataLogger PROPERTIES PREFIX "")

add_executable(DataLoggerComp DataLoggerComp.cpp ${comp_sources})
target_link_libraries(DataLoggerComp ${OPENRTM_LIBRARIES})

set(target DataLogger DataLoggerComp)

install(TARGETS ${target}
  RUNTIME DESTINATION bin CONFIGURATIONS Release Debug
  LIBRARY DESTINATION lib CONFIGURATIONS Release Debug
)

