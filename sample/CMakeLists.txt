# shared object file suffix
if(APPLE)
  set(SO_SFX "dylib")
else()
  set(SO_SFX "so")
endif()

add_subdirectory(PA10)
add_subdirectory(HRP4C)

