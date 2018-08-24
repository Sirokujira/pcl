###############################################################################
# Find the native nanoflann headers and libraries.
#
# This sets the following variables:
# NANOFLANN_FOUND - True if nanoflann was found.
# NANOFLANN_INCLUDE_DIRS - Directories containing the nanoflann include files.
# NANOFLANN_DEFINITIONS - Compiler flags for nanoflann.
# NANOFLANN_VERSION - Package version

# Look for the header file.
find_path(NANOFLANN_INCLUDE_DIR NAMES include/nanoflann.hpp
      HINTS ${PC_NANOFLANN_INCLUDEDIR} ${PC_NANOFLANN_INCLUDE_DIRS} "${NANOFLANN_ROOT}" "$ENV{NANOFLANN_ROOT}"
      PATHS "$ENV{PROGRAMFILES}/nanoflann" "$ENV{PROGRAMW6432}/nanoflann"
      PATH_SUFFIXES include
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(NANOFLANN DEFAULT_MSG NANOFLANN_INCLUDE_DIR)

mark_as_advanced(NANOFLANN_INCLUDE_DIR)

if(NANOFLANN_FOUND)
  # SET(NANOFLANN_INCLUDE_DIRS ${NANOFLANN_INCLUDE_DIR})
  message(STATUS "nanoflann found (include: ${NANOFLANN_INCLUDE_DIR})")
endif(NANOFLANN_FOUND)

