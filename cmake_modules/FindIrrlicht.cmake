FIND_PATH(
IRRLICHT_DIR
NAMES include/irrlicht/irrlicht.h
PATHS /usr /usr/local
DOC "the top directory of irrlicht")

IF ( IRRLICHT_DIR )
    MESSAGE(STATUS "Found Irrlicht in ${IRRLICHT_DIR}")
    set(IRRLICHT_INCLUDE_DIR ${IRRLICHT_DIR}/include)
    if (APPLE)
      set(IRRLICHT_LIBRARIES "Irrlicht -framework Cocoa -framework Carbon -framework IOKit -framework OpenGL")
    else()
      set(IRRLICHT_LIBRARIES Irrlicht)
    endif()
    set(IRRLICHT_FOUND true)
ELSE ( IRRLICHT_INCLUDE_DIR )
    MESSAGE(STATUS "Irrlicht not found")
    set(IRRLICHT_FOUND false)
ENDIF ( IRRLICHT_DIR )
