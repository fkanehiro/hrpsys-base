add_executable(hrpsys-self-collision-checker scc.cpp main.cpp)
target_link_libraries(hrpsys-self-collision-checker ${OPENHRP_LIBRARIES})

set(target hrpsys-self-collision-checker)

install(TARGETS ${target}
  RUNTIME DESTINATION bin
)
