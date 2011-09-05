FIND_PATH(
OCTOMAP_DIR
NAMES include/octomap/octomap.h
PATHS /usr /usr/local
DOC "the top directory of octomap")

IF (OCTOMAP_DIR)
    MESSAGE(STATUS "Found Octomap in ${OCTOMAP_DIR}")
    set(OCTOMAP_INCLUDE_DIR ${OCTOMAP_DIR}/include)
    set(OCTOMAP_LIBRARIES octomap octomath)
    set(OCTOMAP_FOUND true)
ELSE ( OCTOMAP_INCLUDE_DIR )
    MESSAGE(STATUS "Octomap not found")
    set(OCTOMAP_FOUND false)
ENDIF ( OCTOMAP_DIR )
