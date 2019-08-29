#set(comp_sources Beeper.cpp BeeperService_impl.cpp ../SoftErrorLimiter/beep.cpp)
set(comp_sources Beeper.cpp ../SoftErrorLimiter/beep.cpp)
set(libs hrpModel-3.1 hrpUtil-3.1 hrpsysBaseStub)
add_library(Beeper SHARED ${comp_sources})
target_link_libraries(Beeper ${libs})
set_target_properties(Beeper PROPERTIES PREFIX "")

add_executable(BeeperComp BeeperComp.cpp ${comp_sources})
target_link_libraries(BeeperComp ${libs})

set(target Beeper BeeperComp)

install(TARGETS ${target}
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
)
