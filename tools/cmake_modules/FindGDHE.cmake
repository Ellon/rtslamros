

FIND_PATH(GDHE_INCLUDE_DIR gdhe/GDHE.h
  ${GDHE_ROOT}/include
  $ENV{GDHE_ROOT}/include
  $ENV{GDHE_ROOT}
  /usr/local/include
  /usr/include
  /opt/local/include
  /sw/local/include
  /sw/include
  NO_DEFAULT_PATH
  )

FIND_LIBRARY(GDHE_LIBRARY
  NAMES "GDHE"
  PATHS
  ${GDHE_ROOT}/lib/Debug
  ${GDHE_ROOT}/lib
  $ENV{GDHE_ROOT}/lib/Debug
  $ENV{GDHE_ROOT}/lib
  NO_DEFAULT_PATH
  )

IF(GDHE_LIBRARY)
  SET(GDHE_FOUND "YES")
ENDIF(GDHE_LIBRARY)