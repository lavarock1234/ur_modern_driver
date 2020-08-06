#.rst:
# Findur_rtde
# -------
#
# Find ur_rtde library
# Report problems to xuchu.ding@gmail.com

include(FindPackageHandleStandardArgs)
set(ur_rtde_FOUND TRUE)
set(ur_rtde_ROOT_DIR "$ENV{BLUEHILL_USR}/ur_rtde-1.2.5")

find_path(ur_rtde_INCLUDE_DIR
  NAMES ur_rtde/rtde.h
  PATHS ${ur_rtde_ROOT_DIR}/include
  NO_DEFAULT_PATH
)

if (NOT ur_rtde_INCLUDE_DIR)
  message(STATUS "Failed to find ur_rtde include dir!")
  set(ur_rtde_FOUND FALSE)
endif()

find_library(ur_rtde_LIBRARY
  NAMES librtde.so
  PATHS ${ur_rtde_ROOT_DIR}/lib
  NO_DEFAULT_PATH
)

if (NOT ur_rtde_LIBRARY)
  message(STATUS "Failed to find ur_rtde library!")
  set(ur_rtde_FOUND FALSE)
endif()

if (ur_rtde_FOUND)
  set(ur_rtde_INCLUDE_DIRS ${ur_rtde_INCLUDE_DIR})
  file(GLOB ur_rtde_LIBRARIES ${ur_rtde_LIBRARY})
  message(STATUS "Found libur_rtde (include: ${ur_rtde_INCLUDE_DIRS} ${ur_rtde_LIBRARIES})")
endif()

find_package_handle_standard_args(ur_rtde REQUIRED_VARS ur_rtde_LIBRARIES ur_rtde_INCLUDE_DIRS)
