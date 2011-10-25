set(idl_flags -bcxx -Wbuse_quotes -Wbh=.hh -Wbs=Sk.cpp -Wba -Wbd=DynSk.cpp -I${OPENRTM_DIR}/include/rtm/idl)
set(package_path OpenHRP)

macro(generate_stub_skel idl_basename)

  set(idl_file ${CMAKE_CURRENT_SOURCE_DIR}/../../idl/${idl_basename}.idl)
  set(jarfile ${CMAKE_CURRENT_BINARY_DIR}/${idl_basename}.jar)

  add_custom_command(
    OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/${idl_basename}.hh ${CMAKE_CURRENT_BINARY_DIR}/${idl_basename}Sk.cpp ${CMAKE_CURRENT_BINARY_DIR}/${idl_basename}DynSk.cpp ${jarfile} ${CMAKE_CURRENT_BINARY_DIR}/../../python/${idl_basename}_idl.py
    COMMAND omniidl ${idl_flags} ${idl_file}
    COMMAND omniidl -bpython -C../../python -I${OPENRTM_DIR}/include/rtm/idl ${idl_file}
    COMMAND ${IDLJ} ${idlj_flags} -I${OPENRTM_DIR}/include/rtm/idl ${idl_file}
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

macro(generate_stub_skel_pkg idl_basename package_path)

  set(idl_file ${CMAKE_CURRENT_SOURCE_DIR}/../../idl/${idl_basename}.idl)
  set(jarfile ${CMAKE_CURRENT_BINARY_DIR}/${idl_basename}.jar)

  add_custom_command(
    OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/${idl_basename}.hh ${CMAKE_CURRENT_BINARY_DIR}/${idl_basename}Sk.cpp ${CMAKE_CURRENT_BINARY_DIR}/${idl_basename}DynSk.cpp ${jarfile} #${CMAKE_CURRENT_BINARY_DIR}/../../python/${idl_basename}_idl.py
    COMMAND omniidl ${idl_flags} ${idl_file}
#    COMMAND omniidl -bpython -C../../python ${idl_file}
    COMMAND ${IDLJ} ${idlj_flags} -I${OPENRTM_DIR}/include/rtm/idl ${idl_file}
    COMMAND mkdir -p ${CMAKE_CURRENT_BINARY_DIR}/cls
    COMMAND mkdir -p ${CMAKE_CURRENT_BINARY_DIR}/cls/${package_path}
    COMMAND ${JAVAC} ${javac_flags} src/${package_path}*.java
    COMMAND ${JAR} cf ${jarfile} -C ${CMAKE_CURRENT_BINARY_DIR}/cls ${package_path}.
    DEPENDS ${idl_file}
    )

  include_directories(${CMAKE_CURRENT_BINARY_DIR})

  install(FILES ${jarfile} DESTINATION share/hrpsys/jar)
#  install(FILES ${CMAKE_CURRENT_BINARY_DIR}/../../python/${idl_basename}_idl.py DESTINATION share/python)
#  install(FILES ${CMAKE_CURRENT_BINARY_DIR}/../../python/OpenHRP/__init__.py DESTINATION share/python/OpenHRP)
#  install(FILES ${CMAKE_CURRENT_BINARY_DIR}/../../python/OpenHRP__POA/__init__.py DESTINATION share/python/OpenHRP__POA)

endmacro()

add_subdirectory(NullComponent)
add_subdirectory(RobotHardware)
add_subdirectory(StateHolder)
add_subdirectory(WavPlayer)
add_subdirectory(SequencePlayer)
add_subdirectory(DataLogger)
add_subdirectory(Joystick)
add_subdirectory(Joystick2Velocity2D)
add_subdirectory(Joystick2PanTiltAngles)
add_subdirectory(ForwardKinematics)
add_subdirectory(HGcontroller)
add_subdirectory(JpegDecoder)

# Octomap
find_package(Octomap)
if (OCTOMAP_FOUND)
  add_subdirectory(OccupancyGridMap3D)
endif()
