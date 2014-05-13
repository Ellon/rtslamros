# Find the header files

FIND_PATH(JAFAR_INCLUDE_DIR jafar/kernel/jafarMacro.hpp
  ${JAFAR_ROOT}/include
  $ENV{JAFAR_ROOT}/include
  $ENV{JAFAR_ROOT}
  /usr/local/include
  /usr/include
  /opt/local/include
  /sw/local/include
  /sw/include
  NO_DEFAULT_PATH
  )

# Macro to unify finding both the debug and release versions of the
# libraries; this is adapted from the OpenSceneGraph FIND_LIBRARY
# macro.

MACRO(FIND_JAFAR_LIBRARY MYLIBRARY MYLIBRARYNAME)

  FIND_LIBRARY("${MYLIBRARY}_DEBUG"
    NAMES "jafar-${MYLIBRARYNAME}_d"
    PATHS
    ${JAFAR_ROOT}/lib/Debug
    ${JAFAR_ROOT}/lib
    $ENV{JAFAR_ROOT}/lib/Debug
    $ENV{JAFAR_ROOT}/lib
    NO_DEFAULT_PATH
    )

  FIND_LIBRARY("${MYLIBRARY}_DEBUG"
    NAMES "jafar-${MYLIBRARYNAME}_d"
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
  
  FIND_LIBRARY(${MYLIBRARY}
    NAMES "jafar-${MYLIBRARYNAME}"
    PATHS
    ${JAFAR_ROOT}/lib/Release
    ${JAFAR_ROOT}/lib
    $ENV{JAFAR_ROOT}/lib/Release
    $ENV{JAFAR_ROOT}/lib
    NO_DEFAULT_PATH
    )

  FIND_LIBRARY(${MYLIBRARY}
    NAMES "jafar-${MYLIBRARYNAME}"
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
  
  IF(NOT ${MYLIBRARY}_DEBUG)
    IF(MYLIBRARY)
      SET(${MYLIBRARY}_DEBUG ${MYLIBRARY})
    ENDIF(MYLIBRARY)
  ENDIF( NOT ${MYLIBRARY}_DEBUG)
  
ENDMACRO(FIND_JAFAR_LIBRARY LIBRARY LIBRARYNAME)

# Find the jafar elements
FIND_JAFAR_LIBRARY(JAFAR_KERNEL_LIBRARY kernel)
FIND_JAFAR_LIBRARY(JAFAR_JMATH_LIBRARY jmath)
FIND_JAFAR_LIBRARY(JAFAR_CORREL_LIBRARY correl)
FIND_JAFAR_LIBRARY(JAFAR_GDHE_LIBRARY gdhe)
FIND_JAFAR_LIBRARY(JAFAR_IMAGE_LIBRARY image)
FIND_JAFAR_LIBRARY(JAFAR_QDISPLAY_LIBRARY qdisplay)
FIND_JAFAR_LIBRARY(JAFAR_RTSLAM_LIBRARY rtslam)

# Jafar itself declared found if we found the kernel and jmath libraries 
SET(JAFAR_FOUND "NO")
IF(JAFAR_KERNEL_LIBRARY AND JAFAR_JMATH_LIBRARY)
  SET(JAFAR_FOUND "YES")
ENDIF(JAFAR_KERNEL_LIBRARY AND JAFAR_JMATH_LIBRARY)
