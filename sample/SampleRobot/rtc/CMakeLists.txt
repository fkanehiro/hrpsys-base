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
set(libs hrpsysBaseStub)

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
  RUNTIME DESTINATION bin CONFIGURATIONS Release Debug
  LIBRARY DESTINATION lib CONFIGURATIONS Release Debug
)
install(CODE "
execute_process(COMMAND cmake -E create_symlink ../../../lib/SampleComponent.so SampleComponent.so WORKING_DIRECTORY \$ENV{DESTDIR}\${CMAKE_INSTALL_PREFIX}/share/hrpsys/lib)
")
# compile python idl
install(CODE "execute_process(COMMAND omniidl -bpython -C\$ENV{DESTDIR}\${CMAKE_INSTALL_PREFIX}/${python_dist_pkg_dir}/hrpsys -I${OPENRTM_IDL_DIR} SampleComponentService.idl WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})")
install(CODE "execute_process(COMMAND python -m compileall . WORKING_DIRECTORY ${python_dist_pkg_dir}/hrpsys)")
