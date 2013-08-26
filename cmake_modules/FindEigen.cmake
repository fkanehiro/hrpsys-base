IF (UNIX)
    FIND_PATH(
    EIGEN_INCLUDE_DIR
    Eigen/Core
    PATHS /opt/local/include/eigen3 /usr/include/eigen3 /usr/local/include/eigen3 /usr/include/eigen /usr/include/eigen2
    DOC "the top directory of Eigen/Core in Eigen")
ELSE()
    IF (WIN32)
        INCLUDE(${CMAKE_MODULE_PATH}/GetWinSystemPrefixPaths.cmake)
        foreach(localPath ${WIN_SYSTEM_PREFIX_PATHS})
          list(APPEND EnumPaths ${localPath}/Eigen/include/eigen3)
        endforeach()
        FIND_PATH(
        EIGEN_INCLUDE_DIR
        Eigen/Core
        PATHS ${EnumPaths}
        DOC "the top directory of Eigen/Core in Eigen")
    ENDIF(WIN32)
ENDIF(UNIX)

IF ( EIGEN_INCLUDE_DIR )
    MESSAGE(STATUS "Looking for Eigen - found")
    SET(KDL_CFLAGS "${KDL_CFLAGS} -I${EIGEN_INCLUDE_DIR}" CACHE INTERNAL "")
ELSE ( EIGEN_INCLUDE_DIR )
    MESSAGE(FATAL_ERROR "Looking for Eigen - not found")
ENDIF ( EIGEN_INCLUDE_DIR )
