# Locate nvidia-texture-tools
# This module defines
# NVTT_LIBRARY
# NVTT_FOUND, if false, do not try to link to nvtt
# NVTT_INCLUDE_DIR, where to find the headers
#


FIND_PATH(NVTT_INCLUDE_DIR nvtt/nvtt.h
  PATHS
  /usr/local
  /usr
  $ENV{NVTT_DIR}
  ${3rdPartyRoot}
  PATH_SUFFIXES include
)

# NVTT
FIND_LIBRARY(NVTT_LIBRARY_RELEASE
  NAMES nvtt
  PATHS
  /usr/local
  /usr
  $ENV{NVTT_DIR}
  ${3rdPartyRoot}
  PATH_SUFFIXES lib64 lib lib/shared lib/static lib64/static
)

FIND_LIBRARY(NVTT_LIBRARY_DEBUG
  NAMES nvtt_d
  PATHS
  /usr/local
  /usr
  $ENV{NVTT_DIR}
  ${3rdPartyRoot}
  PATH_SUFFIXES lib64 lib lib/shared lib/static lib64/static
)

# NVIMAGE
FIND_LIBRARY(NVIMAGE_LIBRARY_RELEASE
  NAMES nvimage
  PATHS
  /usr/local
  /usr
  $ENV{NVTT_DIR}
  ${3rdPartyRoot}
  PATH_SUFFIXES lib64 lib lib/shared lib/static lib64/static
)

FIND_LIBRARY(NVIMAGE_LIBRARY_DEBUG
  NAMES nvimage_d
  PATHS
  /usr/local
  /usr
  $ENV{NVTT_DIR}
  ${3rdPartyRoot}
  PATH_SUFFIXES lib64 lib lib/shared lib/static lib64/static
)

# NVMATH
FIND_LIBRARY(NVMATH_LIBRARY_RELEASE
  NAMES nvmath
  PATHS
  /usr/local
  /usr
  $ENV{NVTT_DIR}
  ${3rdPartyRoot}
  PATH_SUFFIXES lib64 lib lib/shared lib/static lib64/static
)

FIND_LIBRARY(NVMATH_LIBRARY_DEBUG
  NAMES nvmath_d
  PATHS
  /usr/local
  /usr
  $ENV{NVTT_DIR}
  ${3rdPartyRoot}
  PATH_SUFFIXES lib64 lib lib/shared lib/static lib64/static
)

# NVCORE
FIND_LIBRARY(NVCORE_LIBRARY_RELEASE
  NAMES nvcore
  PATHS
  /usr/local
  /usr
  $ENV{NVTT_DIR}
  ${3rdPartyRoot}
  PATH_SUFFIXES lib64 lib lib/shared lib/static lib64/static
)
FIND_LIBRARY(NVCORE_LIBRARY_DEBUG
  NAMES nvcore_d
  PATHS
  /usr/local
  /usr
  $ENV{NVTT_DIR}
  ${3rdPartyRoot}
  PATH_SUFFIXES lib64 lib lib/shared lib/static lib64/static
)

# NVTHREAD
FIND_LIBRARY(NVTHREAD_LIBRARY_RELEASE
  NAMES nvthread
  PATHS
  /usr/local
  /usr
  $ENV{NVTT_DIR}
  ${3rdPartyRoot}
  PATH_SUFFIXES lib64 lib lib/shared lib/static lib64/static
)
FIND_LIBRARY(NVTHREAD_LIBRARY_DEBUG
  NAMES nvthread_d
  PATHS
  /usr/local
  /usr
  $ENV{NVTT_DIR}
  ${3rdPartyRoot}
  PATH_SUFFIXES lib64 lib lib/shared lib/static lib64/static
)

# SQUISH
FIND_LIBRARY(NVSQUISH_LIBRARY_RELEASE
  NAMES squish
  PATHS
  /usr/local
  /usr
  $ENV{NVTT_DIR}
  ${3rdPartyRoot}
  PATH_SUFFIXES lib64 lib lib/shared lib/static lib64/static
)
FIND_LIBRARY(NVSQUISH_LIBRARY_DEBUG
  NAMES squish_d
  PATHS
  /usr/local
  /usr
  $ENV{NVTT_DIR}
  ${3rdPartyRoot}
  PATH_SUFFIXES lib64 lib lib/shared lib/static lib64/static
)

# BC6H
FIND_LIBRARY(NVBC6H_LIBRARY_RELEASE
  NAMES bc6h
  PATHS
  /usr/local
  /usr
  $ENV{NVTT_DIR}
  ${3rdPartyRoot}
  PATH_SUFFIXES lib64 lib lib/shared lib/static lib64/static
)
FIND_LIBRARY(NVBC6H_LIBRARY_DEBUG
  NAMES bc6h_d
  PATHS
  /usr/local
  /usr
  $ENV{NVTT_DIR}
  ${3rdPartyRoot}
  PATH_SUFFIXES lib64 lib lib/shared lib/static lib64/static
)

# BC7
FIND_LIBRARY(NVBC7_LIBRARY_RELEASE
  NAMES bc7
  PATHS
  /usr/local
  /usr
  $ENV{NVTT_DIR}
  ${3rdPartyRoot}
  PATH_SUFFIXES lib64 lib lib/shared lib/static lib64/static
)
FIND_LIBRARY(NVBC7_LIBRARY_DEBUG
  NAMES bc7_d
  PATHS
  /usr/local
  /usr
  $ENV{NVTT_DIR}
  ${3rdPartyRoot}
  PATH_SUFFIXES lib64 lib lib/shared lib/static lib64/static
)



SET(FOUND_NVTT_RELEASE_LIBS 
${NVTT_LIBRARY_RELEASE}  
${NVCORE_LIBRARY} 
${NVMATH_LIBRARY_RELEASE} 
${NVIMAGE_LIBRARY_RELEASE} 
${NVTHREAD_LIBRARY_RELEASE} 
${NVBC7_LIBRARY_RELEASE} 
${NVBC6H_LIBRARY_RELEASE} 
${NVSQUISH_LIBRARY_RELEASE}
)

SET(FOUND_NVTT_DEBUG_LIBS 
${NVTT_LIBRARY_DEBUG}
${NVCORE_LIBRARY_DEBUG} 
${NVMATH_LIBRARY_DEBUG} 
${NVIMAGE_LIBRARY_DEBUG} 
${NVTHREAD_LIBRARY_DEBUG} 
${NVBC7_LIBRARY_DEBUG} 
${NVBC6H_LIBRARY_DEBUG} 
${NVSQUISH_LIBRARY_DEBUG}
)

IF(NVTT_LIBRARY_DEBUG)
   SET(NVTT_LIBRARIES optimized "${FOUND_NVTT_RELEASE_LIBS}" debug "${FOUND_NVTT_DEBUG_LIBS}")
   SET(NVTT_LIBRARIES_DEBUG ${NVTT_LIBRARY_DEBUG})
   SET(NVTT_LIBRARIES_RELEASE ${NVTT_LIBRARY_RELEASE})
ELSE(NVTT_LIBRARY_DEBUG)
   SET(NVTT_LIBRARIES ${FOUND_NVTT_RELEASE_LIBS})
   SET(NVTT_LIBRARIES_DEBUG ${FOUND_NVTT_RELEASE_LIBS})
   SET(NVTT_LIBRARIES_RELEASE ${FOUND_NVTT_RELEASE_LIBS})
ENDIF(NVTT_LIBRARY_DEBUG)


SET(NVTT_FOUND "NO")
IF(NVTT_LIBRARY_RELEASE AND NVTT_INCLUDE_DIR)
   SET(NVTT_FOUND "YES" )
ENDIF(NVTT_LIBRARY_RELEASE AND NVTT_INCLUDE_DIR)

MARK_AS_ADVANCED(
  NVTT_INCLUDE_DIR
  NVTT_LIBRARY_RELEASE
  NVTT_LIBRARY_DEBUG
  NVTT_LIBRARIES_DEBUG
  NVTT_LIBRARIES_RELEASE
  ) 
