set(idl_files 
  DataLoggerService.idl 
  StateHolderService.idl 
  RobotHardwareService.idl 
  WavPlayerService.idl 
  SequencePlayerService.idl 
  ForwardKinematicsService.idl
  ExecutionProfileService.idl
  OGMap3DService.idl
  NullService.idl
  EmergencyStopperService.idl
  TimeKeeperService.idl
  Img.idl
  HRPDataTypes.idl
  pointcloud.idl
  ImpedanceControllerService.idl
  SoftErrorLimiterService.idl
  CollisionDetectorService.idl
  GraspControllerService.idl
  TorqueFilterService.idl
  KalmanFilterService.idl
  VirtualForceSensorService.idl
  StabilizerService.idl
  AutoBalancerService.idl
  RemoveForceSensorLinkOffsetService.idl
  ServoControllerService.idl
  TorqueControllerService.idl
  ThermoLimiterService.idl
  ReferenceForceUpdaterService.idl
  AccelerationFilterService.idl
  )

set(openhrp_idl_files
  ClockGenerator.idl
  CollisionDetector.idl
  Controller.idl
  DynamicsSimulator.idl
  ModelLoader.idl
  OnlineViewer.idl
  OpenHRPCommon.idl
  ViewSimulator.idl
  World.idl
  )
if(NOT QNXNTO)
  list(APPEND openhrp_idl_files  PathPlanner.idl)
endif()

# add CMAKE_CURRENT_SOURCE_DIR to all files
set(EXTRA_IDL_FILES "" CACHE PATH "directories of extra idls") # this must contain paths
set(idl_files2 ${idl_files})
foreach(idl_file ${idl_files})
  list(APPEND idl_files_path ${CMAKE_CURRENT_SOURCE_DIR}/${idl_file})
endforeach()
set(idl_files ${idl_files_path} ${EXTRA_IDL_FILES})

set(idl_flags -bcxx -Wbuse_quotes -Wbh=.hh -Wbs=Sk.cpp -Wba -Wbd=DynSk.cpp -I${OPENRTM_IDL_DIR})

string(REGEX REPLACE ";" " " idl_files2 "${idl_files2}")
string(REGEX REPLACE ";" " " openhrp_idl_files2 "${openhrp_idl_files}")
configure_file(${PROJECT_SOURCE_DIR}/cpack_modules/postinst.in ${PROJECT_BINARY_DIR}/postinst)

macro(generate_cpp_stub idl_file idl_basename)
  add_custom_command(
    OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/${idl_basename}.hh ${CMAKE_CURRENT_BINARY_DIR}/${idl_basename}Sk.cpp ${CMAKE_CURRENT_BINARY_DIR}/${idl_basename}DynSk.cpp
    COMMAND omniidl ${idl_flags} ${idl_file}
    DEPENDS ${idl_file}
    )
endmacro()

macro(generate_java_stub idl_file idl_basename)
  add_custom_command(
    OUTPUT ${idl_basename}.compiled src
    COMMAND ${IDLJ} ${idlj_flags} -I${OPENRTM_IDL_DIR} ${idl_file}
    COMMAND ${CMAKE_COMMAND} -E echo "" > ${idl_basename}.compiled
    DEPENDS ${idl_file}
    )
endmacro()

foreach(idl_file ${idl_files})
  get_filename_component(idl_basename ${idl_file} NAME_WE)
  generate_cpp_stub(${idl_file} ${idl_basename})
  list(APPEND sources ${idl_basename}Sk.cpp ${idl_basename}DynSk.cpp)
  list(APPEND headers ${CMAKE_CURRENT_BINARY_DIR}/${idl_basename}.hh)
  if(COMPILE_JAVA_STUFF)
    generate_java_stub(${idl_file} ${idl_basename})
  endif()
  list(APPEND compiled ${idl_basename}.compiled)
endforeach()

set(jarfile ${CMAKE_CURRENT_BINARY_DIR}/hrpsysBaseStub.jar)

add_custom_command(
  OUTPUT ${jarfile} cls
  COMMAND mkdir -p cls
  COMMAND ${JAVAC} ${javac_flags} src/*/*.java -d cls
  COMMAND ${JAR} cf ${jarfile} -C ${CMAKE_CURRENT_BINARY_DIR}/cls Img -C ${CMAKE_CURRENT_BINARY_DIR}/cls OpenHRP -C ${CMAKE_CURRENT_BINARY_DIR}/cls OpenRTM -C ${CMAKE_CURRENT_BINARY_DIR}/cls RTC -C ${CMAKE_CURRENT_BINARY_DIR}/cls _SDOPackage
  COMMAND cp ${jarfile} ${CMAKE_SOURCE_DIR}/plugin
  DEPENDS ${compiled}
  )
if(COMPILE_JAVA_STUFF)
  add_custom_target(StubJava ALL DEPENDS ${jarfile})
endif()

set(target hrpsysBaseStub)
add_library(${target} SHARED ${sources})
target_link_libraries(${target} ${OPENRTM_LIBRARIES})

install(TARGETS ${target}
	LIBRARY DESTINATION lib
)

if(COMPILE_JAVA_STUFF)
  install(FILES ${jarfile} DESTINATION share/java)
endif()
foreach(_idl_file ${idl_files})
  message("Check if ${_idl_file} exists")
  if(NOT EXISTS ${_idl_file})
    message(FATAL_ERROR "ERROR: ${_idl_file} is not present")
  endif()
endforeach()
foreach(_idl_file ${openhrp_idl_files})
  message("Check if ${OPENHRP_IDL_DIR}/OpenHRP/${_idl_file} exists")
  if(NOT EXISTS ${OPENHRP_IDL_DIR}/OpenHRP/${_idl_file})
    message(FATAL_ERROR "ERROR: ${OPENHRP_IDL_DIR}/OpenHRP/${_idl_file} is not present")
  endif()
endforeach()

install(CODE "execute_process(COMMAND omniidl -bpython -C\$ENV{DESTDIR}\${CMAKE_INSTALL_PREFIX}/${python_dist_pkg_dir}/hrpsys -I${OPENRTM_IDL_DIR} ${idl_files} WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})")
install(CODE "execute_process(COMMAND omniidl -bpython -C\$ENV{DESTDIR}\${CMAKE_INSTALL_PREFIX}/${python_dist_pkg_dir}/hrpsys -I${OPENRTM_IDL_DIR} ${openhrp_idl_files} WORKING_DIRECTORY ${OPENHRP_IDL_DIR}/OpenHRP)")
install(CODE "execute_process(COMMAND python -m compileall . WORKING_DIRECTORY ${python_dist_pkg_dir}/hrpsys)")
install(FILES ${headers} DESTINATION include/hrpsys/idl)
install(FILES ${idl_files} DESTINATION share/hrpsys/idl)

