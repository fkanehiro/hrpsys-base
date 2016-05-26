#for solo compile
set(libs hrpsysBaseStub)
if(NOT OPENRTM_IDL_DIR)
  execute_process(
    COMMAND rtm-config --rtm-idldir
    OUTPUT_VARIABLE OPENRTM_IDL_DIR
    OUTPUT_STRIP_TRAILING_WHITESPACE)
  find_package(PkgConfig)
  pkg_check_modules(hrpsys hrpsys-base)
  pkg_check_modules(openhrp3 openhrp3.1)
  set(OPENHRP_DIR ${openhrp3_PREFIX})
  include_directories(${OPENRTM_IDL_DIR};${hrpsys_INCLUDE_DIRS})
  link_directories(${hrpsys_LIBRARY_DIRS})
  add_definitions(-DHRPSYS_PACKAGE_VERSION=\"\\"${hrpsys_VERSION}\\"\")
  set(libs ${libs} ${hrpsys_LIBRARIES})
  execute_process(
    COMMAND python -c "from distutils import sysconfig; print sysconfig.get_config_var(\"VERSION\")"
    OUTPUT_VARIABLE PYTHON_VERSION
    OUTPUT_STRIP_TRAILING_WHITESPACE)
  set(python_dist_pkg_dir lib/python${PYTHON_VERSION}/dist-packages)
endif()
# idl
set(idl_flags -bcxx -Wbuse_quotes -Wbh=.hh -Wbs=Sk.cpp -Wba -Wbd=DynSk.cpp -I${OPENRTM_IDL_DIR})
add_custom_command(
  OUTPUT SampleComponentService.hh SampleComponentServiceSk.cpp SampleComponentServiceDynSk.cpp
  COMMAND omniidl ${idl_flags} ${CMAKE_CURRENT_SOURCE_DIR}/SampleComponentService.idl
  DEPENDS SampleComponentService.idl
  )
add_custom_target(SampleComponentService.idl.compiled ALL DEPENDS SampleComponentService.hh SampleComponentServiceSk.cpp SampleComponentServiceDynSk.cpp)

# plugins comps
set(comp_sources SampleComponent.cpp SampleComponent_impl.cpp SampleComponentServiceSk.cpp SampleComponentServiceDynSk.cpp)

add_library(SampleComponent SHARED ${comp_sources})
target_link_libraries(SampleComponent ${libs})
set_target_properties(SampleComponent PROPERTIES PREFIX "")
include_directories(${CMAKE_CURRENT_BINARY_DIR})

add_executable(SampleComponentComp SampleComponentComp.cpp ${comp_sources})
target_link_libraries(SampleComponentComp ${libs})

set(target SampleComponent SampleComponentComp)

add_dependencies(SampleComponent SampleComponentService.hh)
add_dependencies(SampleComponentComp SampleComponentService.hh)

configure_file(samplerobot_sample_component.py.in ${CMAKE_CURRENT_BINARY_DIR}/samplerobot_sample_component.py)

install(PROGRAMS
  ${CMAKE_CURRENT_BINARY_DIR}/samplerobot_sample_component.py
  DESTINATION share/hrpsys/samples/SampleRobot/rtc
)

install(TARGETS ${target}
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
)
install(CODE "
execute_process(COMMAND cmake -E create_symlink ../../../lib/SampleComponent.so SampleComponent.so WORKING_DIRECTORY \$ENV{DESTDIR}\${CMAKE_INSTALL_PREFIX}/share/hrpsys/lib)
")
# compile python idl
install(CODE "execute_process(COMMAND omniidl -bpython -C\$ENV{DESTDIR}\${CMAKE_INSTALL_PREFIX}/${python_dist_pkg_dir}/hrpsys -I${OPENRTM_IDL_DIR} SampleComponentService.idl WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})")
install(CODE "execute_process(COMMAND python -m compileall . WORKING_DIRECTORY ${python_dist_pkg_dir}/hrpsys)")
