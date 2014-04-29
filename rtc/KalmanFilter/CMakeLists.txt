option(BUILD_KALMAN_FILTER "Build KalmanFilter RTC" ON)

if(NOT BUILD_KALMAN_FILTER)
  return()
endif()

set(comp_sources KalmanFilter.cpp KalmanFilterService_impl.cpp)
set(libs hrpModel-3.1 hrpsysBaseStub)

include_directories(${PROJECT_SOURCE_DIR}/rtc/KalmanFilter/kalman)

add_library(KalmanFilter SHARED ${comp_sources})
target_link_libraries(KalmanFilter ${libs})
set_target_properties(KalmanFilter PROPERTIES PREFIX "")

add_executable(KalmanFilterComp KalmanFilterComp.cpp ${comp_sources})
target_link_libraries(KalmanFilterComp ${libs})

set(target KalmanFilter KalmanFilterComp)

install(TARGETS ${target}
  RUNTIME DESTINATION bin CONFIGURATIONS Release Debug
  LIBRARY DESTINATION lib CONFIGURATIONS Release Debug
)
