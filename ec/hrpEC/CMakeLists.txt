set(common_libs ${OPENRTM_LIBRARIES} hrpIo hrpsysBaseStub)
find_package(ART REQUIRED)

link_directories(${LIBIO_DIR})
set(target hrpEC)
if (ART_LINUX)
  add_library(hrpEC SHARED hrpEC-art.cpp hrpEC-common.cpp /usr/lib/art_syscalls.o)
else()
  add_library(hrpEC SHARED hrpEC.cpp hrpEC-common.cpp)
endif()

if (APPLE OR QNXNTO)
  target_link_libraries(hrpEC ${common_libs})
else()
  target_link_libraries(hrpEC ${common_libs} rt)
endif()
set_target_properties(hrpEC PROPERTIES PREFIX "")

install(TARGETS ${target}
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
)


