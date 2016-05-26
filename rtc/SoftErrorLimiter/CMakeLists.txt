set(comp_sources SoftErrorLimiter.cpp SoftErrorLimiterService_impl.cpp robot.cpp beep.cpp JointLimitTable.cpp)
set(libs hrpModel-3.1 hrpUtil-3.1 hrpsysBaseStub)
add_library(SoftErrorLimiter SHARED ${comp_sources})
target_link_libraries(SoftErrorLimiter ${libs})
set_target_properties(SoftErrorLimiter PROPERTIES PREFIX "")

add_executable(SoftErrorLimiterComp SoftErrorLimiterComp.cpp ${comp_sources})
target_link_libraries(SoftErrorLimiterComp ${libs})

set(target SoftErrorLimiter SoftErrorLimiterComp)

install(TARGETS ${target}
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
)
