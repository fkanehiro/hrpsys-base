message(STATUS "USE_QPOASES=${USE_QPOASES}")
if(USE_QPOASES)
  add_subdirectory(qpOASES)
endif()
