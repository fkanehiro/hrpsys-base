cmake_minimum_required(VERSION 2.4)

if(COMMAND cmake_policy)
  cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

set(CMAKE_ALLOW_LOOSE_LOOP_CONSTRUCTS true)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules/)

include(FindPkgConfig)
include(CheckIncludeFiles)

option(ENABLE_INSTALL_RPATH "Enable RPATH setting for installed binary files" OFF)

if(ENABLE_INSTALL_RPATH)
  SET(CMAKE_SKIP_BUILD_RPATH  FALSE)
  SET(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE) 
  SET(CMAKE_INSTALL_RPATH "$ORIGIN/../lib")
  SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH FALSE)
else()
  SET(CMAKE_SKIP_BUILD_RPATH  FALSE)
  SET(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE) 
  SET(CMAKE_INSTALL_RPATH "")
  SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH FALSE)
endif()


# commands
if(UNIX)
  set(RMDIR rm -fr)
  set(CPSEP ":")
elseif(WIN32)
  set(RMDIR rmdir /S/Q)
  set(CPSEP ";")
endif()

# shell script file suffix
if(UNIX)
  set(SH_SFX "sh")
elseif(WIN32)
  set(SH_SFX "bat")
endif()

# OpenRTM-aist(>= 1.0.0)    
find_package(OpenRTM REQUIRED)

# OpenHRP
find_package(OpenHRP REQUIRED)
include_directories(${OPENHRP_INCLUDE_DIRS})
link_directories(${OPENHRP_LIBRARY_DIRS})
if(UNIX)
    add_definitions(${OPENHRP_DEFINITIONS})
endif()

include_directories(${PROJECT_SOURCE_DIR}/lib)
include_directories(${CMAKE_BINARY_DIR}/idl)

set(JAVAC javac)
set(JAR jar)
set(JAR jar)
set(IDLJ idlj)
set(idlj_flags -fclient -fserver -emitAll -td src)
set(javac_flags -target 1.5 -d . -sourcepath src)

if(NOT CMAKE_BUILD_TYPE)
  set(
    CMAKE_BUILD_TYPE Release CACHE STRING
    "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
    FORCE)
endif()

set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)

set(CPACK_PACKAGE_NAME hrpsys-base)
set(CPACK_PACKAGE_VENDOR "AIST")
set(CPACK_PACKAGE_CONTACT "Fumio Kanehiro <f-kanehiro@aist.go.jp>")
set(CPACK_PACKAGE_VERSION_MAJOR 3)
set(CPACK_PACKAGE_VERSION_MINOR 1)
set(CPACK_PACKAGE_VERSION_PATCH 4)
set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE i386)
set(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_CURRENT_SOURCE_DIR}/LICENSE-en.txt")
set(CPACK_PACKAGE_VERSION ${CPACK_PACKAGE_VERSION_MAJOR}.${CPACK_PACKAGE_VERSION_MINOR}.${CPACK_PACKAGE_VERSION_PATCH} )
set(CPACK_PACKAGE_FILE_NAME "${CPACK_PACKAGE_NAME}-${CPACK_PACKAGE_VERSION}${OPENHRP_VERSION_ADD_TAIL}_${CPACK_DEBIAN_PACKAGE_ARCHITECTURE}")
set(CPACK_DEBIAN_PACKAGE_DEPENDS "libxml2 libcv4 libcvaux4 libhighgui4 libsdl1.2debian libglut3 libxmu6 libirrlicht1.7")

if(UNIX)
  set(CPACK_PACKAGING_INSTALL_PREFIX "/usr" CACHE PATH "The root directory of the package")
endif()

include(CPack)

find_package(LibXml2 REQUIRED)
find_package(SDL REQUIRED)
find_package(OpenGL REQUIRED)
include(FindPkgConfig)
pkg_check_modules(OpenCV opencv)
if (NOT OpenCV_FOUND)
  pkg_check_modules(OpenCV opencv-2.3.1)
  if (NOT OpenCV_FOUND)
    message(FATAL_ERROR "opencv not found")
  endif()
endif()
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
find_package(GLUT REQUIRED)
find_package(QuickHull REQUIRED)

configure_file(hrpsys-base.pc.in ${CMAKE_CURRENT_BINARY_DIR}/hrpsys-base.pc @ONLY)
install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/hrpsys-base.pc
  DESTINATION lib/pkgconfig)

add_subdirectory(jython)
add_subdirectory(python)
add_subdirectory(idl)
add_subdirectory(lib)
add_subdirectory(ec)
add_subdirectory(rtc)
add_subdirectory(doc)
add_subdirectory(util)
add_subdirectory(sample)
