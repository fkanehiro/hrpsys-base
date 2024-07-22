find_file(
    QHULL_DIR
    NAMES libqhull/libqhull.h qhull.h
    HINTS "${QHULL_ROOT}" "$ENV{QHULL_ROOT}" "${QHULL_INCLUDE_DIR}"
    PATHS "$ENV{PROGRAMFILES}/QHull" "$ENV{PROGRAMW6432}/QHull"
          "$ENV{PROGRAMFILES}/qhull 6.2.0.1373"
          "$ENV{PROGRAMW6432}/qhull 6.2.0.1373"
    PATH_SUFFIXES qhull src/libqhull libqhull include)

IF ( QHULL_DIR )
    MESSAGE(STATUS "Found Qhull in ${QHULL_DIR}")
    set(QHULL_INCLUDE_DIR ${QHULL_DIR})
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
