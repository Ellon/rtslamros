# Find the header files

FIND_PATH(Jafar_INCLUDE_DIR kernel/jafarMacro.hpp
  ${JAFAR_ROOT}/include/jafar
  $ENV{JAFAR_ROOT}/include/jafar
  $ENV{JAFAR_ROOT}
  $ENV{ROBOTPKG_BASE}/include/jafar
  /usr/local/include
  /usr/include
  /opt/local/include
  /sw/local/include
  /sw/include
  NO_DEFAULT_PATH
  )

IF(Jafar_INCLUDE_DIR)
  LIST(APPEND Jafar_INCLUDE_DIRS ${Jafar_INCLUDE_DIR})
ENDIF()

# Macro to unify finding both the debug and release versions of the
# libraries; this is adapted from the OpenSceneGraph FIND_LIBRARY
# macro.

MACRO(FIND_JAFAR_MODULE MYMODULE)

string(TOLOWER ${MYMODULE} MYMODULELOWER)

FIND_PATH(Jafar_${MYMODULE}_INCLUDE_DIR ${MYMODULELOWER}Exception.hpp
  ${Jafar_INCLUDE_DIR}/${MYMODULELOWER}
  /usr/local/include
  /usr/include
  /opt/local/include
  /sw/local/include
  /sw/include
  NO_DEFAULT_PATH
  )

  FIND_LIBRARY(Jafar_${MYMODULE}_LIBRARY
    NAMES "jafar-${MYMODULELOWER}"
    PATHS
    ${JAFAR_ROOT}/lib/Release
    ${JAFAR_ROOT}/lib
    $ENV{JAFAR_ROOT}/lib/Release
    $ENV{JAFAR_ROOT}/lib
    $ENV{ROBOTPKG_BASE}/lib/Release
    $ENV{ROBOTPKG_BASE}/lib
    NO_DEFAULT_PATH
    )

  FIND_LIBRARY(Jafar_${MYMODULE}_LIBRARY
    NAMES "jafar-${MYMODULELOWER}"
    PATHS
    ~/Library/Frameworks
    /Library/Frameworks
    /usr/local/lib
    /usr/local/lib64
    /usr/lib
    /usr/lib64
    /opt/local/lib
    /sw/local/lib
    /sw/lib
    )
  
  IF(Jafar_${MYMODULE}_INCLUDE_DIR AND Jafar_${MYMODULE}_LIBRARY)
     SET(Jafar_${MYMODULE}_FOUND TRUE)
  ENDIF()

ENDMACRO(FIND_JAFAR_MODULE MYMODULE)

# Find the jafar elements
IF(Jafar_INCLUDE_DIR)
  FOREACH(MODULE ${Jafar_FIND_COMPONENTS})
    FIND_JAFAR_MODULE(${MODULE})
    IF(Jafar_${MODULE}_FOUND)
      LIST(APPEND Jafar_LIBRARIES ${Jafar_${MODULE}_LIBRARY})
      LIST(APPEND Jafar_INCLUDE_DIRS ${Jafar_${MODULE}_INCLUDE_DIR})
    ENDIF()
  ENDFOREACH()
ENDIF()

# Jafar itself declared found if we found the kernel and jmath libraries 
IF(Jafar_LIBRARIES AND Jafar_INCLUDE_DIRS)
   SET(Jafar_FOUND TRUE)
   IF(NOT Jafar_FIND_QUIETLY)
      MESSAGE(STATUS "Found the following Jafar modules:")
   ENDIF(NOT Jafar_FIND_QUIETLY)
   FOREACH(MODULE ${Jafar_FIND_COMPONENTS})
      IF( Jafar_${MODULE}_FOUND )
         IF(NOT Jafar_FIND_QUIETLY)
            MESSAGE(STATUS "  ${MODULE}")
         ENDIF(NOT Jafar_FIND_QUIETLY)
      ELSE( Jafar_${MODULE}_FOUND )
         IF(Jafar_FIND_REQUIRED_${MODULE})
            MESSAGE(FATAL_ERROR "  The following Jafar module could not be found:\n      ${MODULE}\n  You may need to install this additional Jafar module. Alternatively, set JAFAR_ROOT to the location of Jafar")
         ENDIF(Jafar_FIND_REQUIRED_${MODULE})
      ENDIF( Jafar_${MODULE}_FOUND )
   ENDFOREACH(MODULE)
ENDIF()

IF((NOT Jafar_FOUND) AND Jafar_FIND_REQUIRED)
   MESSAGE(FATAL_ERROR "  Jafar not found!\n\nYou may need to install Jafar kernel module. Alternatively, set JAFAR_ROOT to the location of Jafar")
ENDIF((NOT Jafar_FOUND) AND Jafar_FIND_REQUIRED)
