generate_stub_skel(SequencePlayerService "OpenHRP")

set(comp_sources interpolator.cpp timeUtil.cpp seqplay.cpp SequencePlayer.cpp SequencePlayerService_impl.cpp SequencePlayerServiceSk.cpp )
add_library(SequencePlayer SHARED ${comp_sources})
target_link_libraries(SequencePlayer ${OPENRTM_LIBRARIES} hrpModel-3.1 hrpCollision-3.1 hrpUtil-3.1)
set_target_properties(SequencePlayer PROPERTIES PREFIX "")

add_executable(SequencePlayerComp SequencePlayerComp.cpp ${comp_sources})
target_link_libraries(SequencePlayerComp ${OPENRTM_LIBRARIES} hrpModel-3.1 hrpCollision-3.1 hrpUtil-3.1)

set(target SequencePlayer SequencePlayerComp)

install(TARGETS ${target}
  RUNTIME DESTINATION bin CONFIGURATIONS Release Debug
  LIBRARY DESTINATION lib CONFIGURATIONS Release Debug
)

