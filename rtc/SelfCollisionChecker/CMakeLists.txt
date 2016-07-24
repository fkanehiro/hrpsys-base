set(comp_sources SelfCollisionChecker.cpp)
set(libs hrpModel-3.1 hrpCollision-3.1 hrpUtil-3.1 hrpsysBaseStub)
add_library(SelfCollisionChecker SHARED ${comp_sources})
target_link_libraries(SelfCollisionChecker ${libs})
set_target_properties(SelfCollisionChecker PROPERTIES PREFIX "")

add_executable(SelfCollisionCheckerComp SelfCollisionCheckerComp.cpp ${comp_sources})
target_link_libraries(SelfCollisionCheckerComp ${libs})

set(target SelfCollisionChecker SelfCollisionCheckerComp)

install(TARGETS ${target}
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
)
