include_directories(${LIBXML2_INCLUDE_DIR})
add_executable(ProjectGenerator ProjectGenerator.cpp)
target_link_libraries(ProjectGenerator ${OPENHRP_LIBRARIES} ${LIBXML2_LIBRARIES})

install(TARGETS ${target}
  RUNTIME DESTINATION bin CONFIGURATIONS Release Debug
)
