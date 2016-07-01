cmake_minimum_required(VERSION 2.4)
project(hrpsys)

enable_testing()
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

# Specify include directory.
#   After https://github.com/fkanehiro/hrpsys-base/pull/842, hrpsys-base local include files are used as "hrpsys/xx/yy.h".
#   For this reason, we make ${CMAKE_BINARY_DIR}/hrpsys directory and generate symbolic link for related include files under it.
#   To deal with include files in ${CMAKE_BINARY_DIR}/hrpsys as hrpsys-base local include files, we set cmake's include_directories before other include_directories setting to set higher priority for ${CMAKE_BINARY_DIR}/hrpsys.
execute_process(
  COMMAND mkdir -p ${CMAKE_BINARY_DIR}/hrpsys
)
if (NOT EXISTS ${CMAKE_BINARY_DIR}/hrpsys/io)
  execute_process(
    COMMAND ln -s ${PROJECT_SOURCE_DIR}/lib/io ${CMAKE_BINARY_DIR}/hrpsys/io
   )
endif()
if (NOT EXISTS ${CMAKE_BINARY_DIR}/hrpsys/util)
  execute_process(
    COMMAND ln -s ${PROJECT_SOURCE_DIR}/lib/util ${CMAKE_BINARY_DIR}/hrpsys/util
   )
endif()
if (NOT EXISTS ${CMAKE_BINARY_DIR}/hrpsys/idl)
  execute_process(
    COMMAND ln -s ${CMAKE_BINARY_DIR}/idl ${CMAKE_BINARY_DIR}/hrpsys/idl
   )
endif()
include_directories(${CMAKE_BINARY_DIR})

# OpenRTM-aist(>= 1.0.0)
find_package(OpenRTM REQUIRED)
if(EXISTS "${OPENRTM_DIR}/include/openrtm-1.1/rtm/RTObjectStateMachine.h")
add_definitions(-DOPENRTM_VERSION_TRUNK)
endif()

# OpenHRP
find_package(OpenHRP REQUIRED)
include_directories(${OPENHRP_INCLUDE_DIRS})
link_directories(${OPENHRP_LIBRARY_DIRS})
if(UNIX)
    add_definitions(${OPENHRP_DEFINITIONS})
endif()

if(ENABLE_INSTALL_RPATH)
  set(CMAKE_INSTALL_RPATH "$ORIGIN/../lib:$ORIGIN/../../../lib:${OPENHRP_LIBRARY_DIRS}")
endif()

option(COMPILE_JAVA_STUFF "Compile java stuff" OFF)
if(COMPILE_JAVA_STUFF)
  set(JAVAC javac)
  set(JAR jar)
  set(IDLJ idlj)
  set(idlj_flags -DORBIT2_IDL -fclient -fserver -emitAll -td src)
  if (NOT DEFINED Java_VERSION_MAJOR)
    execute_process(COMMAND ${JAVAC} -version ERROR_VARIABLE jvar ERROR_STRIP_TRAILING_WHITESPACE)
    STRING( REGEX REPLACE "javac " "" Java_VERSION_STRING "${jvar}" )
    # following regex pulled from cmake-2.8 FindJava.cmake
    STRING( REGEX REPLACE "([0-9]+).*"          "\\1" Java_VERSION_MAJOR "${Java_VERSION_STRING}" )
    STRING( REGEX REPLACE "[0-9]+\\.([0-9]+).*" "\\1" Java_VERSION_MINOR "${Java_VERSION_STRING}" )
  endif()
  set(javac_flags -target ${Java_VERSION_MAJOR}.${Java_VERSION_MINOR} -d . -sourcepath src)
endif()

if(NOT CMAKE_BUILD_TYPE OR CMAKE_BUILD_TYPE STREQUAL None)
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
set(CPACK_PACKAGE_VERSION_MINOR 9)
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
if ( NOT SDL_FOUND )
  message(WARNING "Package SDL is not found!")
endif( NOT SDL_FOUND )
find_package(OpenGL)
if ( NOT OPENGL_FOUND )
  message(WARNING "Package OpenGL is not found!")
endif( NOT OPENGL_FOUND )
find_package(GLUT)
if ( NOT GLUT_FOUND )
  message(WARNING "Package GLUT is not found!")
endif( NOT GLUT_FOUND )
include(FindPkgConfig)
pkg_check_modules(OpenCV opencv)
if (NOT OpenCV_FOUND)
  pkg_check_modules(OpenCV opencv-2.3.1)
  if (NOT OpenCV_FOUND)
    message(WARNING "opencv not found")
  endif()
endif()
pkg_check_modules(GLEW glew)
if (NOT GLEW_FOUND)
  message(WARNING "Package GLEW is not found")
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
set(python_dist_pkg_dir lib/python${PYTHON_VERSION}/dist-packages)

configure_file(hrpsys-base.pc.in ${CMAKE_CURRENT_BINARY_DIR}/hrpsys-base.pc @ONLY)
install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/hrpsys-base.pc
  DESTINATION lib/pkgconfig)

add_definitions(-DHRPSYS_PACKAGE_VERSION=\"\\"${CPACK_PACKAGE_VERSION_MAJOR}.${CPACK_PACKAGE_VERSION_MINOR}.${CPACK_PACKAGE_VERSION_PATCH}\\"\")

option(NO_DIGITAL_INPUT "Disable readDigitalInput and lengthDigitalInput" OFF)
if(NO_DIGITAL_INPUT)
  add_definitions(-DNO_DIGITAL_INPUT)
  message(STATUS "Disablng readDigitalInput and lengthDigitalInput")
endif()

option(ROBOT_IOB_VERSION "Supported robot IOB version (lib/io/iob.h)")
if("${ROBOT_IOB_VERSION}" STREQUAL "OFF")
  set(ROBOT_IOB_VERSION 2)
endif()
add_definitions(-DROBOT_IOB_VERSION=${ROBOT_IOB_VERSION})
message(STATUS "compile iob with -DROBOT_IOB_VERSION=${ROBOT_IOB_VERSION}")

if(COMPILE_JAVA_STUFF)
  add_subdirectory(jython)
endif()
add_subdirectory(python)
add_subdirectory(idl)
add_subdirectory(lib)
add_subdirectory(ec)
# option for 3rdparty
option(USE_QPOASES "Build qpOASES" ON)
add_subdirectory(3rdparty)
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

#if catkin environment
string(REGEX MATCH "catkin" need_catkin "$ENV{_}")
if(need_catkin OR "${CATKIN_BUILD_BINARY_PACKAGE}")
  find_package(catkin)
  if(catkin_FOUND)
    catkin_package_xml()
    if(NOT hrpsys_VERSION VERSION_EQUAL CPACK_PACKAGE_VERSION)
      message( FATAL_ERROR "hrpsys_version (from package.xml): ${hrpsys_VERSION} is not equal CPACK_PACKAGE_VERSION: ${CPACK_PACKAGE_VERSION}" )
    endif()
  endif()
  install(CODE "file(WRITE \$ENV{DESTDIR}\${CMAKE_INSTALL_PREFIX}/share/doc/hrpsys/rospack_nosubdirs)")
  install(FILES package.xml DESTINATION share/hrpsys/)
  install(CODE "
execute_process(COMMAND cmake -E make_directory lib WORKING_DIRECTORY \$ENV{DESTDIR}\${CMAKE_INSTALL_PREFIX}/share/hrpsys)
file(GLOB targets RELATIVE \${CMAKE_CURRENT_BINARY_DIR}/lib/ \${CMAKE_CURRENT_BINARY_DIR}/lib/*.so)
message(\"\${targets}\")
foreach(target \${targets})
  if(\${target} STREQUAL \"hrpsysext.so\" OR \${target} STREQUAL \"libhrpIo.so\" OR \${target} STREQUAL \"libhrpsysBaseStub.so\" OR \${target} STREQUAL \"libhrpsysUtil.so\")
  else()
    message(\"cmake -E create_symlink ../../../lib/\${target} \${target} WORKING_DIRECTORY \$ENV{DESTDIR}\${CMAKE_INSTALL_PREFIX}/share/hrpsys/lib\")
    execute_process(COMMAND cmake -E create_symlink ../../../lib/\${target} \${target} WORKING_DIRECTORY \$ENV{DESTDIR}\${CMAKE_INSTALL_PREFIX}/share/hrpsys/lib)
  endif()
endforeach()")
  install(CODE "
execute_process(COMMAND cmake -E make_directory \$ENV{DESTDIR}\${CMAKE_INSTALL_PREFIX}/lib/hrpsys)
file(GLOB targets RELATIVE \${CMAKE_CURRENT_BINARY_DIR}/bin/ \${CMAKE_CURRENT_BINARY_DIR}/bin/*)
message(\"\${targets}\")
foreach(target \${targets})
  execute_process(COMMAND cmake -E create_symlink ../../bin/\${target} \${target} WORKING_DIRECTORY \$ENV{DESTDIR}\${CMAKE_INSTALL_PREFIX}/lib/hrpsys)
endforeach()")
  install(DIRECTORY test launch DESTINATION share/hrpsys USE_SOURCE_PERMISSIONS)
  install(DIRECTORY rtc ec DESTINATION share/hrpsys/src USE_SOURCE_PERMISSIONS)
  install(CODE "
execute_process(COMMAND cmake -E make_directory share/hrpsys WORKING_DIRECTORY \$ENV{DESTDIR}\${CMAKE_INSTALL_PREFIX}/share/hrpsys)
execute_process(COMMAND cmake -E create_symlink ../../../hrpsys/idl     share/hrpsys/idl WORKING_DIRECTORY \$ENV{DESTDIR}\${CMAKE_INSTALL_PREFIX}/share/hrpsys)
execute_process(COMMAND cmake -E create_symlink ../../../hrpsys/samples share/hrpsys/samples WORKING_DIRECTORY \$ENV{DESTDIR}\${CMAKE_INSTALL_PREFIX}/share/hrpsys)
")
endif()
if(need_catkin)
  install(CODE
"
## this is tricky force write catkin marker file
set(_catkin_marker_file \"\${CMAKE_INSTALL_PREFIX}/.catkin\")
# check if the develspace marker file exists yet
if(EXISTS \${_catkin_marker_file})
  file(READ \${_catkin_marker_file} _existing_sourcespaces)
  if(_existing_sourcespaces STREQUAL \"\")
    # write this sourcespace to the marker file
    file(WRITE \${_catkin_marker_file} \"${CMAKE_INSTALL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}\")
  else()
    # append to existing list of sourcespaces if it's not in the list
    list(FIND _existing_sourcespaces \"${CMAKE_INSTALL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}\" _existing_sourcespace_index)
    if(_existing_sourcespace_index EQUAL -1)
      file(APPEND \${_catkin_marker_file} \";${CMAKE_INSTALL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}\")
    endif()
  endif()
else()
  # create a new develspace marker file
  # NOTE: extra care must be taken when running multiple catkin jobs in parallel 
  #       so that this does not overwrite the result of a similar call in another package
  file(WRITE \${_catkin_marker_file} \"${CMAKE_INSTALL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}\")
endif()
")
endif()
