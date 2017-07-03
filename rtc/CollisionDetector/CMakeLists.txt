set(seq_dir ${PROJECT_SOURCE_DIR}/rtc/SequencePlayer)
if (USE_HRPSYSUTIL)
  set(comp_sources ${seq_dir}/interpolator.cpp CollisionDetector.cpp CollisionDetectorService_impl.cpp GLscene.cpp ../SoftErrorLimiter/beep.cpp)
  add_definitions(-DUSE_HRPSYSUTIL)
else()
  # BVutil.cpp can be used without hrpsysUtil dependencies
  set(comp_sources ${seq_dir}/interpolator.cpp CollisionDetector.cpp CollisionDetectorService_impl.cpp ../../lib/util/BVutil.cpp ../SoftErrorLimiter/beep.cpp)
  set(libs hrpModel-3.1 hrpCollision-3.1 hrpsysBaseStub)
endif()

if(USE_FCL)
  add_definitions(-DUSE_FCL=${USE_FCL})
  list(APPEND comp_sources FCLLinkPair.cpp)
  pkg_check_modules(fcl fcl REQUIRED)
else()
  list(APPEND comp_sources VclipLinkPair.cpp)
  set(vclip_dir vclip_1.0/)
  set(vclip_sources ${vclip_dir}/src/vclip.C ${vclip_dir}/src/PolyTree.C ${vclip_dir}/src/mv.C)
endif()

add_definitions(-DQHULL)
if(NOT (${CMAKE_SYSTEM_PROCESSOR} MATCHES amd64* OR
      ${CMAKE_SYSTEM_PROCESSOR} MATCHES x86_64*) )
## only for 32bit system
  if(USE_FCL)
    set_source_files_properties(PROPERTIES COMPILE_FLAGS -ffloat-store)
  else()
    set_source_files_properties(${vclip_sources} PROPERTIES COMPILE_FLAGS -ffloat-store)
  endif()
endif()

if(USE_FCL)
  link_directories(${fcl_LIBRARY_DIRS})
  include_directories(${LIBXML2_INCLUDE_DIR} ${QHULL_INCLUDE_DIR} ${seq_dir} ${fcl_INCLUDE_DIR})
  add_library(CollisionDetector SHARED ${comp_sources})
else()
  include_directories(${LIBXML2_INCLUDE_DIR} ${QHULL_INCLUDE_DIR} ${seq_dir} ${vclip_dir}/include)
  add_library(CollisionDetector SHARED ${comp_sources} ${vclip_sources})
endif()

if (USE_HRPSYSUTIL)
  target_link_libraries(CollisionDetector hrpsysUtil ${QHULL_LIBRARIES} ${fcl_LIBRARIES})
else()
  target_link_libraries(CollisionDetector ${QHULL_LIBRARIES} ${libs} ${fcl_LIBRARIES})
endif()
set_target_properties(CollisionDetector PROPERTIES PREFIX "")

if(USE_FCL)
  add_executable(CollisionDetectorComp CollisionDetectorComp.cpp ${comp_sources})
else()
  add_executable(CollisionDetectorComp CollisionDetectorComp.cpp ${comp_sources} ${vclip_sources})
endif()

if (USE_HRPSYSUTIL)
  target_link_libraries(CollisionDetectorComp hrpsysUtil ${QHULL_LIBRARIES} ${fcl_LIBRARIES} boost_system)
else ()
  target_link_libraries(CollisionDetectorComp ${QHULL_LIBRARIES} ${libs} ${fcl_LIBRARIES} boost_system)
endif()

add_executable(SetupCollisionPair SetupCollisionPair.cpp)
target_link_libraries(SetupCollisionPair CollisionDetector ${OPENHRP_LIBRARIES} ${QHULL_LIBRARIES} boost_system)

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
