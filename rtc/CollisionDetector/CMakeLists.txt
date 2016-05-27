set(seq_dir ${PROJECT_SOURCE_DIR}/rtc/SequencePlayer)
if (USE_HRPSYSUTIL)
  set(comp_sources ${seq_dir}/interpolator.cpp CollisionDetector.cpp CollisionDetectorService_impl.cpp GLscene.cpp VclipLinkPair.cpp ../SoftErrorLimiter/beep.cpp)
  add_definitions(-DUSE_HRPSYSUTIL)
else()
  # BVutil.cpp can be used without hrpsysUtil dependencies
  set(comp_sources ${seq_dir}/interpolator.cpp CollisionDetector.cpp CollisionDetectorService_impl.cpp VclipLinkPair.cpp ../../lib/util/BVutil.cpp ../SoftErrorLimiter/beep.cpp)
  set(libs hrpModel-3.1 hrpCollision-3.1 hrpsysBaseStub)
endif()
set(vclip_dir vclip_1.0/)
set(vclip_sources ${vclip_dir}/src/vclip.C ${vclip_dir}/src/PolyTree.C ${vclip_dir}/src/mv.C)

add_definitions(-DQHULL)
if(NOT (${CMAKE_SYSTEM_PROCESSOR} MATCHES amd64* OR
      ${CMAKE_SYSTEM_PROCESSOR} MATCHES x86_64*) )
## only for 32bit system
set_source_files_properties(${vclip_sources} PROPERTIES COMPILE_FLAGS -ffloat-store)
endif()
include_directories(${LIBXML2_INCLUDE_DIR} ${QHULL_INCLUDE_DIR} ${seq_dir} ${vclip_dir}/include)
add_library(CollisionDetector SHARED ${comp_sources} ${vclip_sources})
if (USE_HRPSYSUTIL)
  target_link_libraries(CollisionDetector hrpsysUtil ${QHULL_LIBRARIES})
else()
  target_link_libraries(CollisionDetector ${QHULL_LIBRARIES} ${libs})
endif()
set_target_properties(CollisionDetector PROPERTIES PREFIX "")

add_executable(CollisionDetectorComp CollisionDetectorComp.cpp ${comp_sources} ${vclip_sources})
if (USE_HRPSYSUTIL)
  target_link_libraries(CollisionDetectorComp hrpsysUtil ${QHULL_LIBRARIES})
else ()
  target_link_libraries(CollisionDetectorComp ${QHULL_LIBRARIES} ${libs})
endif()

add_executable(SetupCollisionPair SetupCollisionPair.cpp)
target_link_libraries(SetupCollisionPair CollisionDetector ${OPENHRP_LIBRARIES} ${QHULL_LIBRARIES})

if (USE_HRPSYSUTIL)
  add_executable(CollisionDetectorViewer CollisionDetectorViewer.cpp GLscene.cpp)
  target_link_libraries(CollisionDetectorViewer hrpsysUtil)
  set_target_properties (CollisionDetectorViewer PROPERTIES COMPILE_DEFINITIONS "USE_COLLISION_STATE")
  set(target CollisionDetector CollisionDetectorComp SetupCollisionPair CollisionDetectorViewer)
else()
  set(target CollisionDetector CollisionDetectorComp SetupCollisionPair)
endif()

install(TARGETS ${target}
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
)
