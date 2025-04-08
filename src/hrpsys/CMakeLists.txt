install(PROGRAMS hrpsyspy DESTINATION bin)
install(FILES __init__.py rtm.py waitInput.py DESTINATION ${python_dist_pkg_dir}/hrpsys)
install(PROGRAMS hrpsys_config.py DESTINATION ${python_dist_pkg_dir}/hrpsys)

