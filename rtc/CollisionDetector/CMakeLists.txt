set(seq_dir ${PROJECT_SOURCE_DIR}/rtc/SequencePlayer)
set(comp_sources ${seq_dir}/interpolator.cpp CollisionDetector.cpp GLscene.cpp)
include_directories(${LIBXML2_INCLUDE_DIR} ${QHULL_INCLUDE_DIR} ${seq_dir})
add_library(CollisionDetector SHARED ${comp_sources})
target_link_libraries(CollisionDetector hrpsysUtil)
set_target_properties(CollisionDetector PROPERTIES PREFIX "")

add_executable(CollisionDetectorComp CollisionDetectorComp.cpp ${comp_sources})
target_link_libraries(CollisionDetectorComp hrpsysUtil)

add_executable(SetupCollisionPair SetupCollisionPair.cpp)
target_link_libraries(SetupCollisionPair CollisionDetector ${OPENHRP_LIBRARIES} ${IRRLICHT_LIBRARIES} ${QHULL_LIBRARIES})
set(target CollisionDetector CollisionDetectorComp SetupCollisionPair)

install(TARGETS ${target}
  RUNTIME DESTINATION bin CONFIGURATIONS Release Debug
  LIBRARY DESTINATION lib CONFIGURATIONS Release Debug
)
