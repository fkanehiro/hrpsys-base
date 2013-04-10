set(seq_dir ${PROJECT_SOURCE_DIR}/rtc/SequencePlayer)
set(comp_sources ${seq_dir}/interpolator.cpp CollisionDetector.cpp CollisionDetectorService_impl.cpp VclipLinkPair.cpp)
if (USE_GLBODY)
  set(comp_sources ${comp_sources} GLscene.cpp)
    add_definitions(-DUSE_GLBODY)
endif()
set(vclip_dir vclip_1.0/)
set(vclip_sources ${vclip_dir}/src/vclip.C ${vclip_dir}/src/PolyTree.C ${vclip_dir}/src/mv.C)

add_definitions(-DQHULL)
include_directories(${LIBXML2_INCLUDE_DIR} ${QHULL_INCLUDE_DIR} ${seq_dir} ${vclip_dir}/include)
add_library(CollisionDetector SHARED ${comp_sources} ${vclip_sources})
target_link_libraries(CollisionDetector hrpsysUtil ${QHULL_LIBRARIES})
set_target_properties(CollisionDetector PROPERTIES PREFIX "")

add_executable(CollisionDetectorComp CollisionDetectorComp.cpp ${comp_sources} ${vclip_sources})
target_link_libraries(CollisionDetectorComp hrpsysUtil ${QHULL_LIBRARIES})

add_executable(SetupCollisionPair SetupCollisionPair.cpp)
target_link_libraries(SetupCollisionPair CollisionDetector ${OPENHRP_LIBRARIES} ${QHULL_LIBRARIES})
set(target CollisionDetector CollisionDetectorComp SetupCollisionPair)

install(TARGETS ${target}
  RUNTIME DESTINATION bin CONFIGURATIONS Release Debug
  LIBRARY DESTINATION lib CONFIGURATIONS Release Debug
)
