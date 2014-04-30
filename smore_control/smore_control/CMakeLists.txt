cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
#project(smore_control)
set(CMAKE_CXX_FLAGS "-g -Wall")
set(CMAKE_BUILD_TYPE Debug)

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
endif()


include(FindBoost)
find_package(Boost ${MIN_BOOST_VERSION} REQUIRED system filesystem regex thread)
find_package(Protobuf REQUIRED)
find_package(Threads)
find_package(OpenGL)
find_package(GLUT)

add_subdirectory(extern/glfw-3.0.4)

include_directories(
  extern/glfw-3.0.4/include/
  extern/glew-1.10.0/include/
)

# GLEW Portions
set(GLEW_SOURCE
  extern/glew-1.10.0/src/glew.c
)

set(GLEW_HEADERS)
set(OPENGL_LIBRARY
	${OPENGL_LIBRARY}
	-lGL -lGLU -lXrandr -lXext -lX11 -lrt
	${CMAKE_DL_LIBS}
	${GLFW_LIBRARIES}
)

add_library(GLEW_1100 STATIC
  ${GLEW_SOURCE}
  ${GLEW_INCLUDE}
)
target_link_libraries(GLEW_1100 ${OPENGL_LIBRARY})
# #END of GLEW
 
 
#  PCL Portions
find_package(PCL 1.3 REQUIRED COMPONENTS common io filters)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
# PCL PORTIONS END

set(GLLib ${OPENGL_LIBRARY} ${GLUT_Xi_LIBRARY} ${GLUT_Xmu_LIBRARY} ${GLFW_LIBRARIES} ${OPENGL_glu_LIBRARY} ${GLEW_1100} ${glfw_LIBRARIES}) 

add_library(PCL_OCTREE pcl_octree_impl.cpp)
target_link_libraries(PCL_OCTREE ${PCL_LIBRARIES})

add_library(GLViewer GLViewer.cpp zpr.c)
target_link_libraries(GLViewer glfw GLEW_1100 glut PCL_OCTREE)


include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

include_directories(
        ${GAZEBO_INCLUDE_DIRS}
        ${CMAKE_CURRENT_BINARY_DIR}/msgs
        )
link_directories(${GAZEBO_LIBRARY_DIRS} ${CMAKE_CURRENT_BINARY_DIR}/msgs)
add_subdirectory(msgs)

add_executable(smore_control main.cpp)
target_link_libraries(smore_control command_message collision_message_plus GLViewer ${Boost_LIBRARIES} ${GLLib} ${GAZEBO_LIBRARIES} ${PROTOBUF_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})
add_dependencies(smore_control command_message collision_message_plus)

add_executable(zprdemo zprdemo.c zpr.c)
target_link_libraries(zprdemo glut ${GLLib})
