set(comp_sources CollisionDetector.cpp  IrrModel.cpp)
include_directories(${LIBXML2_INCLUDE_DIR} ${QHULL_INCLUDE_DIR})
add_library(CollisionDetector SHARED ${comp_sources})
target_link_libraries(CollisionDetector ${OPENHRP_LIBRARIES} ${IRRLICHT_LIBRARIES} ${QHULL_LIBRARIES})
set_target_properties(CollisionDetector PROPERTIES PREFIX "")

add_executable(CollisionDetectorComp CollisionDetectorComp.cpp ${comp_sources})
target_link_libraries(CollisionDetectorComp ${OPENHRP_LIBRARIES} ${IRRLICHT_LIBRARIES} ${QHULL_LIBRARIES})

add_executable(SetupCollisionPair SetupCollisionPair.cpp)
target_link_libraries(SetupCollisionPair CollisionDetector ${OPENHRP_LIBRARIES} ${IRRLICHT_LIBRARIES} ${QHULL_LIBRARIES})
set(target CollisionDetector CollisionDetectorComp SetupCollisionPair)

install(TARGETS ${target}
  RUNTIME DESTINATION bin CONFIGURATIONS Release Debug
  LIBRARY DESTINATION lib CONFIGURATIONS Release Debug
)
