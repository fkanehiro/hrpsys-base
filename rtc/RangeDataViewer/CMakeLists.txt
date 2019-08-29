set(comp_sources RangeDataViewer.cpp)
set(libs hrpsysBaseStub ${OpenCV_LIBRARIES})
add_library(RangeDataViewer SHARED ${comp_sources})
target_link_libraries(RangeDataViewer ${libs})
set_target_properties(RangeDataViewer PROPERTIES PREFIX "")

add_executable(RangeDataViewerComp RangeDataViewerComp.cpp ${comp_sources})
target_link_libraries(RangeDataViewerComp ${libs})

set(target RangeDataViewer RangeDataViewerComp)

install(TARGETS ${target}
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
)
