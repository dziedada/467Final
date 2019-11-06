# Find librealsense  https://github.com/IntelRealSense/librealsense
#
# This sets the following variables:
# LIBREALSENSE_FOUND - True if OPENNI was found.
# LIBREALSENSE_INCLUDE_DIRS - Directories containing the OPENNI include files.
# LIBREALSENSE_LIBRARIES - Libraries needed to use OPENNI.
# LIBREALSENSE_DEFINITIONS - Compiler flags for OPENNI.
#
# File forked from augmented_dev, project of alantrrs
# (https://github.com/alantrrs/augmented_dev).

find_package(PkgConfig)
if(${CMAKE_VERSION} VERSION_LESS 2.8.2)
endif()

#add a hint so that it can find it without the pkg-config
find_path(LIBREALSENSE2_INCLUDE_DIR rs.hpp
    HINTS /usr/local/include
    PATH_SUFFIXES librealsense2)
#add a hint so that it can find it without the pkg-config
find_library(LIBREALSENSE2_LIBRARY
    NAMES librealsense2.so
    HINTS /usr/local/lib )

  set(LIBREALSENSE2_INCLUDE_DIRS ${LIBREALSENSE2_INCLUDE_DIR})
  set(LIBREALSENSE2_LIBRARIES ${LIBREALSENSE2_LIBRARY})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(LibRealSense DEFAULT_MSG
  LIBREALSENSE2_LIBRARY LIBREALSENSE2_INCLUDE_DIR)

mark_as_advanced(LIBREALSENSE2_LIBRARY LIBREALSENSE2_INCLUDE_DIR)
