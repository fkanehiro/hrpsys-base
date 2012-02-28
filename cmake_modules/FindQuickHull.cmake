FIND_PATH(
QHULL_DIR
NAMES include/qhull/qhull.h
PATHS /usr /usr/local
DOC "the top directory of qhull")

IF ( QHULL_DIR )
    MESSAGE(STATUS "Found Qhull in ${QHULL_DIR}")
    set(QHULL_INCLUDE_DIR ${QHULL_DIR}/include)
    if (APPLE)
      set(QHULL_LIBRARIES qhull)
    else()
      set(QHULL_LIBRARIES qhull)
    endif()
    set(QHULL_FOUND true)
ELSE ( QHULL_INCLUDE_DIR )
    MESSAGE(STATUS "Qhull not found")
    set(QHULL_FOUND false)
ENDIF ( QHULL_DIR )
