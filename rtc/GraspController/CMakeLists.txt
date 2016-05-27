set(comp_sources GraspController.cpp GraspControllerService_impl.cpp)
set(libs hrpModel-3.1 hrpUtil-3.1 hrpsysBaseStub)
add_library(GraspController SHARED ${comp_sources})
target_link_libraries(GraspController ${libs})
set_target_properties(GraspController PROPERTIES PREFIX "")

add_executable(GraspControllerComp GraspControllerComp.cpp ${comp_sources})
target_link_libraries(GraspControllerComp ${libs})

set(target GraspController GraspControllerComp)

install(TARGETS ${target}
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
)
