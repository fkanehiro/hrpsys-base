set(comp_sources ModifiedServo.cpp)
add_library(ModifiedServo SHARED ${comp_sources})
set(libs hrpModel-3.1 ${OPENRTM_LIBRARIES})
target_link_libraries(ModifiedServo ${libs})
set_target_properties(ModifiedServo PROPERTIES PREFIX "")

add_executable(ModifiedServoComp ModifiedServoComp.cpp ${comp_sources})
target_link_libraries(ModifiedServoComp ${libs})

set(target ModifiedServo ModifiedServoComp)

install(TARGETS ${target}
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
)
