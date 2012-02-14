set(idl_flags -bcxx -Wbuse_quotes -Wbh=.hh -Wbs=Sk.cpp -I${OPENRTM_IDL_DIR})
set(package_path OpenHRP)

macro(generate_stub_skel idl_basename)

  set(idl_file ${CMAKE_CURRENT_SOURCE_DIR}/../../idl/${idl_basename}.idl)
  set(jarfile ${CMAKE_CURRENT_BINARY_DIR}/${idl_basename}.jar)

  add_custom_command(
    OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/${idl_basename}.hh ${CMAKE_CURRENT_BINARY_DIR}/${idl_basename}Sk.cpp ${jarfile} ${CMAKE_CURRENT_BINARY_DIR}/../../python/${idl_basename}_idl.py
    COMMAND omniidl ${idl_flags} ${idl_file}
    COMMAND omniidl -bpython -C../../python -I${OPENRTM_IDL_DIR} ${idl_file}
    COMMAND ${IDLJ} ${idlj_flags} -I${CMAKE_CURRENT_SOURCE_DIR}/../../idl/ -I${OPENRTM_IDL_DIR} ${idl_file}
    COMMAND ${JAVAC} ${javac_flags} src/${package_path}/*.java
    COMMAND ${JAR} cf ${jarfile} -C ${CMAKE_CURRENT_BINARY_DIR} ${package_path}
    COMMAND cp -f ${jarfile} ${PROJECT_SOURCE_DIR}/plugin
    DEPENDS ${idl_file}
    )

  include_directories(${CMAKE_CURRENT_BINARY_DIR})

  install(FILES ${jarfile} DESTINATION share/hrpsys/jar)
  install(FILES ${CMAKE_CURRENT_BINARY_DIR}/../../python/${idl_basename}_idl.py DESTINATION share/hrpsys/python)
  install(FILES ${CMAKE_CURRENT_BINARY_DIR}/../../python/OpenHRP/__init__.py DESTINATION share/hrpsys/python/OpenHRP)
  install(FILES ${CMAKE_CURRENT_BINARY_DIR}/../../python/OpenHRP__POA/__init__.py DESTINATION share/hrpsys/python/OpenHRP__POA)

endmacro()

generate_stub_skel(ExecutionProfileService)

if (EXISTS /usr/lib/art_syscalls.o AND USE_ART)
   add_library(hrpEC SHARED hrpEC.cpp ExecutionProfileServiceSk.cpp /usr/lib/art_syscalls.o)
   add_definitions(-DUSE_ART)
   target_link_libraries(hrpEC ${OPENRTM_LIBRARIES} hrpIo)
else()
   add_library(hrpEC SHARED hrpEC.cpp ExecutionProfileServiceSk.cpp)
   if (APPLE)
      target_link_libraries(hrpEC ${OPENRTM_LIBRARIES} hrpIo)
   else()
      target_link_libraries(hrpEC ${OPENRTM_LIBRARIES} hrpIo rt)
   endif()
endif()

set_target_properties(hrpEC PROPERTIES PREFIX "")

set(target hrpEC)

install(TARGETS ${target}
  RUNTIME DESTINATION bin CONFIGURATIONS Release Debug
  LIBRARY DESTINATION lib CONFIGURATIONS Release Debug
)


