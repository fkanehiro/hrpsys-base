file(GLOB model_files RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.wrl)
install(FILES ${model_files} DESTINATION share/hrpsys/samples/environments)

