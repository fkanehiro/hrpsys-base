set(comp_sources IIRFilter.cpp TorqueFilter.cpp)
set(libs hrpModel-3.1 hrpUtil-3.1 hrpsysBaseStub)
add_library(TorqueFilter SHARED ${comp_sources})
target_link_libraries(TorqueFilter ${libs})
set_target_properties(TorqueFilter PROPERTIES PREFIX "")

add_executable(TorqueFilterComp TorqueFilterComp.cpp ${comp_sources})
target_link_libraries(TorqueFilterComp ${libs})

add_executable(testIIRFilter testIIRFilter.cpp IIRFilter.cpp)
target_link_libraries(testIIRFilter ${libs})

set(target TorqueFilter TorqueFilterComp testIIRFilter)

add_test(testIIRFilterDoubleTest0 testIIRFilter --double --test0 --use-gnuplot false)
add_test(testIIRFilterVector3Test0 testIIRFilter --double --test0 --use-gnuplot false)

install(TARGETS ${target}
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
)
