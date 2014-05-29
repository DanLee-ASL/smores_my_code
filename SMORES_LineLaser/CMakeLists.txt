project(smores_linelaser)
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
endif()

include_directories(
	/usr/include/OGRE/
	/usr/include/OGRE/Paging
)

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(smores_linelaser SHARED LineLaser.cc)
# add_library(smores_linelaser SHARED raytest.cc)
target_link_libraries(smores_linelaser ${GAZEBO_libraries})
