set(comp_sources scc.cpp SelfCollisionChecker.cpp)
set(libs hrpModel-3.1 hrpCollision-3.1 hrpUtil-3.1 hrpsysBaseStub)
add_library(SelfCollisionChecker SHARED ${comp_sources})
target_link_libraries(SelfCollisionChecker ${libs})
set_target_properties(SelfCollisionChecker PROPERTIES PREFIX "")

add_executable(SelfCollisionCheckerComp SelfCollisionCheckerComp.cpp ${comp_sources})
target_link_libraries(SelfCollisionCheckerComp ${libs})

add_executable(hrpsys-self-collision-checker scc.cpp main.cpp)
target_link_libraries(hrpsys-self-collision-checker ${libs})

set(target SelfCollisionChecker SelfCollisionCheckerComp hrpsys-self-collision-checker)

install(TARGETS ${target}
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
)
