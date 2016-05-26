set(comp_sources RangeNoiseMixer.cpp)
set(libs hrpsysBaseStub)
add_library(RangeNoiseMixer SHARED ${comp_sources})
target_link_libraries(RangeNoiseMixer ${libs})
set_target_properties(RangeNoiseMixer PROPERTIES PREFIX "")

add_executable(RangeNoiseMixerComp RangeNoiseMixerComp.cpp ${comp_sources})
target_link_libraries(RangeNoiseMixerComp ${libs})

set(target RangeNoiseMixer RangeNoiseMixerComp)

install(TARGETS ${target}
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
)
