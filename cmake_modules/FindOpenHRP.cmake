set(OPENHRP_FOUND TRUE)

execute_process(
  COMMAND pkg-config --modversion openhrp3.1
  OUTPUT_VARIABLE OPENHRP_VERSION
  RESULT_VARIABLE RESULT
  OUTPUT_STRIP_TRAILING_WHITESPACE)

if(NOT RESULT EQUAL 0)
  set(OPENHRP_FOUND FALSE)
endif()

execute_process(
  COMMAND pkg-config --variable=prefix openhrp3.1
  OUTPUT_VARIABLE OPENHRP_DIR
  RESULT_VARIABLE RESULT
  OUTPUT_STRIP_TRAILING_WHITESPACE)

if(NOT RESULT EQUAL 0)
  set(OPENHRP_FOUND FALSE)
endif()

execute_process(
  COMMAND pkg-config --cflags openhrp3.1
  OUTPUT_VARIABLE OPENHRP_CXX_FLAGS
  RESULT_VARIABLE RESULT
  OUTPUT_STRIP_TRAILING_WHITESPACE)

if(RESULT EQUAL 0)
  string(REGEX MATCHALL "-D.*[^ ;]+" OPENHRP_DEFINITIONS ${OPENHRP_CXX_FLAGS})
else()
  set(OPENHRP_FOUND FALSE)
endif()

execute_process(
  COMMAND pkg-config --cflags-only-I openhrp3.1
  OUTPUT_VARIABLE OPENHRP_INCLUDE_DIRS
  RESULT_VARIABLE RESULT
  OUTPUT_STRIP_TRAILING_WHITESPACE)

if(RESULT EQUAL 0)
  string(REGEX REPLACE "-I" ";" OPENHRP_INCLUDE_DIRS ${OPENHRP_INCLUDE_DIRS})
else()
  set(OPENHRP_FOUND FALSE)
endif()

execute_process(
  COMMAND pkg-config --libs openhrp3.1
  OUTPUT_VARIABLE OPENHRP_LIBRARIES
  RESULT_VARIABLE RESULT
  OUTPUT_STRIP_TRAILING_WHITESPACE)

if(RESULT EQUAL 0)
  string(REGEX MATCHALL "-L[^ ;]+" OPENHRP_LIBRARY_DIRS ${OPENHRP_LIBRARIES})
  string(REGEX REPLACE "-L" ";" OPENHRP_LIBRARY_DIRS "${OPENHRP_LIBRARY_DIRS}")

  string(REGEX REPLACE "-L[^ ;]+" "" OPENHRP_LIBRARIES ${OPENHRP_LIBRARIES})
  separate_arguments(OPENHRP_LIBRARIES)
else()
  set(OPENHRP_FOUND FALSE)
endif()

if(NOT OPENHRP_FOUND)
  set(OPENHRP_DIR NOT_FOUND)
endif()

execute_process(
  COMMAND pkg-config --variable=idl_dir openhrp3.1
  OUTPUT_VARIABLE OPENHRP_IDL_DIR
  RESULT_VARIABLE RESULT
  OUTPUT_STRIP_TRAILING_WHITESPACE)

if(NOT RESULT EQUAL 0)
  set(OPENHRP_FOUND FALSE)
endif()

set(OPENHRP_DIR ${OPENHRP_DIR} CACHE PATH "The top directory of OpenHRP")

if(OPENHRP_FOUND)
  if(NOT OpenHRP_FIND_QUIETLY)
    message(STATUS "Found OpenHRP ${OPENHRP_VERSION} in ${OPENHRP_DIR}")
  endif()
else()
  if(NOT OpenHRP_FIND_QUIETLY)
    if(OpenHRP_FIND_REQUIRED)
      message(FATAL_ERROR "OpenHRP required, please specify it's location.")
    endif()
  endif()
endif()
