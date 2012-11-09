set(common_libs ${OPENRTM_LIBRARIES} hrpIo hrpsysBaseStub)

link_directories(${LIBIO_DIR})
if (EXISTS /usr/lib/art_syscalls.o)
  add_library(hrpEC-art SHARED hrpEC-art.cpp hrpEC-common.cpp /usr/lib/art_syscalls.o)
  target_link_libraries(hrpEC-art ${common_libs})
  set_target_properties(hrpEC-art PROPERTIES PREFIX "")
  set(target hrpEC hrpEC-art)
else()
  set(target hrpEC)
endif()

add_library(hrpEC SHARED hrpEC.cpp hrpEC-common.cpp)
if (APPLE)
  target_link_libraries(hrpEC ${common_libs})
else()
  target_link_libraries(hrpEC ${common_libs} rt)
endif()
set_target_properties(hrpEC PROPERTIES PREFIX "")

install(TARGETS ${target}
  RUNTIME DESTINATION bin CONFIGURATIONS Release Debug
  LIBRARY DESTINATION lib CONFIGURATIONS Release Debug
)


