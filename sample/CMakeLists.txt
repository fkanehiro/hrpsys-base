# shared object file suffix
if(APPLE)
  set(SO_SFX "dylib")
else()
  set(SO_SFX "so")
endif()

add_subdirectory(PA10)
add_subdirectory(HRP4C)

install(PROGRAMS
  simTest1.py simTest2.py simTest3.py simTest4.py simTest5.py
  DESTINATION share/hrpsys/samples)
