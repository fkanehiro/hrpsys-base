set(common_libs ${OPENRTM_LIBRARIES} hrpIo hrpsysBaseStub)
if (EXISTS /usr/lib/art_syscalls.o AND USE_ART)
   add_library(hrpEC SHARED hrpEC.cpp /usr/lib/art_syscalls.o)
   add_definitions(-DUSE_ART)
   target_link_libraries(hrpEC ${common_libs})
else()
   add_library(hrpEC SHARED hrpEC.cpp)
   if (APPLE)
      target_link_libraries(hrpEC ${common_libs})
   else()
      target_link_libraries(hrpEC ${common_libs} rt)
   endif()
endif()

set_target_properties(hrpEC PROPERTIES PREFIX "")

set(target hrpEC)

install(TARGETS ${target}
  RUNTIME DESTINATION bin CONFIGURATIONS Release Debug
  LIBRARY DESTINATION lib CONFIGURATIONS Release Debug
)


