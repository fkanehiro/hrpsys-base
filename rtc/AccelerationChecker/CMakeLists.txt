set(comp_sources AccelerationChecker.cpp)
set(libs hrpsysBaseStub)
add_library(AccelerationChecker SHARED ${comp_sources})
target_link_libraries(AccelerationChecker ${libs})
set_target_properties(AccelerationChecker PROPERTIES PREFIX "")

add_executable(AccelerationCheckerComp AccelerationCheckerComp.cpp ${comp_sources})
target_link_libraries(AccelerationCheckerComp ${libs})

set(target AccelerationChecker AccelerationCheckerComp)

install(TARGETS ${target}
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
)
