cmake_minimum_required(VERSION 2.4)

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


option(USE_ART "Use ART-Linux as operating system of the robot" OFF)

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

include_directories(${PROJECT_SOURCE_DIR}/hrplib)

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
set(CPACK_PACKAGE_VERSION_PATCH 1)
set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE i386)
set(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_CURRENT_SOURCE_DIR}/LICENSE-en.txt")
set(CPACK_PACKAGE_VERSION ${CPACK_PACKAGE_VERSION_MAJOR}.${CPACK_PACKAGE_VERSION_MINOR}.${CPACK_PACKAGE_VERSION_PATCH} )
set(CPACK_PACKAGE_FILE_NAME "${CPACK_PACKAGE_NAME}-${CPACK_PACKAGE_VERSION}${OPENHRP_VERSION_ADD_TAIL}_${CPACK_DEBIAN_PACKAGE_ARCHITECTURE}")
#set(CPACK_DEBIAN_PACKAGE_DEPENDS "openhrp3.1")
include(CPack)

add_subdirectory(jython)
add_subdirectory(python)
add_subdirectory(hrplib)
add_subdirectory(ec)
add_subdirectory(rtc)
add_subdirectory(doc)
add_subdirectory(idl)
add_subdirectory(sample)
