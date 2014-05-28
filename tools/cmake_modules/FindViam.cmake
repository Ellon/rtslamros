

FIND_PATH(Viam_INCLUDE_DIR viam/viamlib.h
  ${VIAM_ROOT}/include
  $ENV{VIAM_ROOT}/include
  $ENV{VIAM_ROOT}
  $ENV{ROBOTPKG_BASE}/include
  /usr/local/include
  /usr/include
  /opt/local/include
  /sw/local/include
  /sw/include
  NO_DEFAULT_PATH
  )

FIND_LIBRARY(Viam_LIBRARY
  NAMES "viam"
  PATHS
  ${VIAM_ROOT}/lib/Debug
  ${VIAM_ROOT}/lib
  $ENV{VIAM_ROOT}/lib/Debug
  $ENV{VIAM_ROOT}/lib
  $ENV{ROBOTPKG_BASE}/lib/Debug
  $ENV{ROBOTPKG_BASE}/lib
  NO_DEFAULT_PATH
  )

IF(Viam_LIBRARY AND Viam_INCLUDE_DIR)
  SET(Viam_FOUND TRUE)
ENDIF(Viam_LIBRARY AND Viam_INCLUDE_DIR)

IF((NOT Viam_FOUND) AND Viam_FIND_REQUIRED)
   MESSAGE(FATAL_ERROR "  Viam not found!")
ENDIF((NOT Viam_FOUND) AND Viam_FIND_REQUIRED)
