cmake_minimum_required(VERSION 2.4)

if(COMMAND cmake_policy)
  cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

set(CMAKE_ALLOW_LOOSE_LOOP_CONSTRUCTS true)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules/)

include(FindPkgConfig)
include(CheckIncludeFiles)

option(ENABLE_INSTALL_RPATH "Enable RPATH setting for installed binary files" OFF)

set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

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

if(ENABLE_INSTALL_RPATH)
  set(CMAKE_INSTALL_RPATH "$ORIGIN/../lib:$ORIGIN/../../../lib:${OPENHRP_LIBRARY_DIRS}")
endif()

option(COMPILE_JAVA_STUFF "Compile java stuff" ON)
if(COMPILE_JAVA_STUFF)
  set(JAVAC javac)
  set(JAR jar)
  set(IDLJ idlj)
  set(idlj_flags -fclient -fserver -emitAll -td src)
  if (NOT DEFINED Java_VERSION_MAJOR)
    execute_process(COMMAND ${JAVAC} -version ERROR_VARIABLE jvar ERROR_STRIP_TRAILING_WHITESPACE)
    STRING( REGEX REPLACE "javac " "" Java_VERSION_STRING "${jvar}" )
    # following regex pulled from cmake-2.8 FindJava.cmake
    STRING( REGEX REPLACE "([0-9]+).*"          "\\1" Java_VERSION_MAJOR "${Java_VERSION_STRING}" )
    STRING( REGEX REPLACE "[0-9]+\\.([0-9]+).*" "\\1" Java_VERSION_MINOR "${Java_VERSION_STRING}" )
  endif()
  set(javac_flags -target ${Java_VERSION_MAJOR}.${Java_VERSION_MINOR} -d . -sourcepath src)
endif()

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
set(CPACK_PACKAGE_VERSION_MAJOR 315)
set(CPACK_PACKAGE_VERSION_MINOR 1)
set(CPACK_PACKAGE_VERSION_PATCH 0)
execute_process(
  COMMAND date +%Y%m%d%H%M
  OUTPUT_VARIABLE CPACK_PACKAGE_DATE
  RESULT_VARIABLE RESULT
  OUTPUT_STRIP_TRAILING_WHITESPACE)
set(CPACK_PACKAGE_VERSION ${CPACK_PACKAGE_VERSION_MAJOR}.${CPACK_PACKAGE_VERSION_MINOR}.${CPACK_PACKAGE_VERSION_PATCH}-${CPACK_PACKAGE_DATE})
set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE i386)
set(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_CURRENT_SOURCE_DIR}/LICENSE-en.txt")
set(CPACK_PACKAGE_FILE_NAME "${CPACK_PACKAGE_NAME}-${CPACK_PACKAGE_VERSION}${OPENHRP_VERSION_ADD_TAIL}_${CPACK_DEBIAN_PACKAGE_ARCHITECTURE}")
#set(CPACK_DEBIAN_PACKAGE_DEPENDS "libxml2,libcv4,libcvaux4,libhighgui4,libsdl1.2debian,libglut3,libxmu6,libirrlicht1.7")
set(CPACK_DEBIAN_PACKAGE_CONTROL_EXTRA postinst)
if(UNIX)
  set(CPACK_PACKAGING_INSTALL_PREFIX "/usr" CACHE PATH "The root directory of the package")
endif()
include(CPack)

find_package(LibXml2 REQUIRED)
find_package(QuickHull REQUIRED)
find_package(SDL)
find_package(OpenGL)
find_package(GLUT)
include(FindPkgConfig)
pkg_check_modules(OpenCV opencv)
pkg_check_modules(GLEW glew)
if (NOT OpenCV_FOUND)
  pkg_check_modules(OpenCV opencv-2.3.1)
  if (NOT OpenCV_FOUND)
    message(WARNING "opencv not found")
  endif()
endif()
if(OpenCV_FOUND AND OPENGL_FOUND AND GLEW_FOUND AND GLUT_FOUND AND SDL_FOUND)
  set(USE_HRPSYSUTIL true)
else()
  set(USE_HRPSYSUTIL false)
endif()

include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
if(NOT QNXNTO)
  find_package(Eigen REQUIRED)
  include_directories(${Eigen_INCLUDE_DIRS})
endif()

execute_process(
  COMMAND python -c "from distutils import sysconfig; print sysconfig.get_config_var(\"VERSION\")"
  OUTPUT_VARIABLE PYTHON_VERSION
  OUTPUT_STRIP_TRAILING_WHITESPACE)
set(python_dist_pkg_dir ${CMAKE_INSTALL_PREFIX}/lib/python${PYTHON_VERSION}/dist-packages)

configure_file(hrpsys-base.pc.in ${CMAKE_CURRENT_BINARY_DIR}/hrpsys-base.pc @ONLY)
install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/hrpsys-base.pc
  DESTINATION lib/pkgconfig)

if(COMPILE_JAVA_STUFF)
  add_subdirectory(jython)
endif()
add_subdirectory(python)
add_subdirectory(idl)
add_subdirectory(lib)
add_subdirectory(ec)
add_subdirectory(rtc)
option(ENABLE_DOXYGEN "Use Doxygen" ON)
if(ENABLE_DOXYGEN)
  add_subdirectory(doc)
endif()
if (NOT QNXNTO)
  add_subdirectory(util)
endif()
add_subdirectory(sample)
add_subdirectory(test)
