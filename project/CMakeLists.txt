cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(cs290i_project)

find_package(PCL 1.7 REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
#find_package(libfreenect REQUIRED)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/")
set(LIBFREENECT_LIBRARIES "/usr/lib/libfreenect.so")

find_package(libusb-1.0 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
include_directories(${LIBUSB_1_INCLUDE_DIR})
include_directories(${OPENGL_INCLUDE_DIRS})
include_directories(${GLUT_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#add_executable (cs290i_project region_growing.cpp)
add_executable (cs290i_project kinect.cpp primitives.cpp)
target_link_libraries (cs290i_project ${PCL_LIBRARIES} ${GLUT_LIBRARY} ${OPENGL_LIBRARIES} ${LIBFREENECT_LIBRARIES})
