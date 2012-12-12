# check if art is enabled

if(NOT ART_LINUX AND CMAKE_SYSTEM_NAME MATCHES "Linux")
  string(RANDOM RANDOM_NAME)
  set(FILE_NAME "check_art_${RANDOM_NAME}.c")
  file(WRITE /tmp/${FILE_NAME} "// this file is generated from FindArt.cmake
#include <linux/art_task.h>
int main () {
  if (art_enter(ART_PRIO_MAX, ART_TASK_PERIODIC, 1000) == -1){
    return -1;
  }
  return 0;
}
")
  exec_program(gcc ARGS "-o /tmp/${RANDOM_NAME} /tmp/${FILE_NAME} /usr/lib/art_syscalls.o"
    OUTPUT_VARIABLE COMPILE_ART_PROGRAM_OUTPUT
    RETURN_VALUE COMPILE_ART_PROGRAM)
  exec_program(/tmp/${RANDOM_NAME}
    OUTPUT_VARIABLE EXECUTE_ART_PROGRAM_OUTPUT
    RETURN_VALUE EXECUTE_ART_PROGRAM)
  file(REMOVE /tmp/${RANDOM_NAME})

  if(COMPILE_ART_PROGRAM EQUAL 0 AND EXECUTE_ART_PROGRAM EQUAL 0)
    message("-- Use ART Linux Thread")
    set(ART_LINUX TRUE)
  else(COMPILE_ART_PROGRAM EQUAL 0 AND EXECUTE_ART_PROGRAM EQUAL 0)
    message("-- Use Normal Linux Thread")
    set(ART_LINUX FALSE)
  endif(COMPILE_ART_PROGRAM EQUAL 0 AND EXECUTE_ART_PROGRAM EQUAL 0)
endif(NOT ART_LINUX AND CMAKE_SYSTEM_NAME MATCHES "Linux")

