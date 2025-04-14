option(BUILD_KALMAN_FILTER "Build KalmanFilter RTC" ON)

if(NOT BUILD_KALMAN_FILTER)
  return()
endif()

set(comp_sources KalmanFilter.cpp KalmanFilterService_impl.cpp)
set(libs hrpModel-${OPENHRP_LIBRARY_VERSION} hrpUtil-${OPENHRP_LIBRARY_VERSION} hrpsysBaseStub)

include_directories(${PROJECT_SOURCE_DIR}/rtc/KalmanFilter/kalman)

add_library(KalmanFilter SHARED ${comp_sources})
target_link_libraries(KalmanFilter ${libs})
set_target_properties(KalmanFilter PROPERTIES PREFIX "")

add_executable(KalmanFilterComp KalmanFilterComp.cpp ${comp_sources})
target_link_libraries(KalmanFilterComp ${libs})
add_executable(testKFilter testKFilter.cpp)
target_link_libraries(testKFilter ${libs})
add_executable(testKalmanFilterEstimation testKalmanFilterEstimation.cpp)
target_link_libraries(testKalmanFilterEstimation ${libs})

set(target KalmanFilter KalmanFilterComp testKalmanFilterEstimation testKFilter)

install(TARGETS ${target}
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
)
