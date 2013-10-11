configure_file(hrpsyspy.in ${CMAKE_CURRENT_BINARY_DIR}/hrpsyspy)

install(PROGRAMS ${CMAKE_CURRENT_BINARY_DIR}/hrpsyspy DESTINATION bin)
install(FILES __init__.py rtm.py waitInput.py hrpsys_config.py DESTINATION ${python_dist_pkg_dir}/hrpsys)

