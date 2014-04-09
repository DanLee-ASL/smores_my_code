cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
#project(smore_control)
set(CMAKE_CXX_FLAGS "-g -Wall")

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
endif()

include(FindBoost)
find_package(Boost ${MIN_BOOST_VERSION} REQUIRED system filesystem regex)
find_package(Protobuf REQUIRED)
find_package(Threads)

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

include_directories(
        ${GAZEBO_INCLUDE_DIRS}
        ${CMAKE_CURRENT_BINARY_DIR}/msgs
        )
link_directories(${GAZEBO_LIBRARY_DIRS} ${CMAKE_CURRENT_BINARY_DIR}/msgs)
add_subdirectory(msgs)

add_executable(smore_control main.cpp)
target_link_libraries(smore_control command_message collision_message_plus ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${PROTOBUF_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})
add_dependencies(smore_control command_message collision_message_plus)
