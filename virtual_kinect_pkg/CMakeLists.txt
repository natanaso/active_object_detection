cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

# define actions
rosbuild_find_ros_package(actionlib_msgs)
include(${actionlib_msgs_PACKAGE_PATH}/cmake/actionbuild.cmake)
genaction()

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
############################################## (END) #################################

#####################################################################
#### LIBRARY INCLUDES
#####################################################################
find_package(VTK REQUIRED)
if (VTK_FOUND)
  message(STATUS "VTK FOUND")
  include_directories(${VTK_INCLUDE_DIRS}) 
  link_directories(${VTK_LIBRARY_DIRS})
else()
  message(STATUS "ERROR: VTK NOT FOUND")
endif()

#find_package(PCL 1.5 REQUIRED)
#if (PCL_FOUND)
#  message(STATUS "PCL FOUND")
#  include_directories(${PCL_INCLUDE_DIRS}) # sets path to PCL headers
#  link_directories(${PCL_LIBRARY_DIRS}) # sets path to PCL libraries
#  add_definitions(${PCL_DEFINITIONS}) # lists needed preprocessor definitions and compiler flags
#else()
#  message(STATUS "ERROR: PCL NOT FOUND")
#endif()

#####################################################################
#### LIBRARY BUILDS
#####################################################################
# library for the virtual kinect offline
rosbuild_add_library(vkin src/vkin_offline.cpp)
target_link_libraries(vkin vtkCommon vtkIO)

#####################################################################
#### EXECUTABLE BUILDS
#####################################################################
# executable for the ros node
rosbuild_add_executable(virtual_kinect_server src/start_vkin_serv.cpp src/virtual_kinect.cpp src/vkin_offline.cpp)
target_link_libraries( virtual_kinect_server vtkCommon vtkIO)

rosbuild_add_executable(test_vkin_server src/test_virtual_kinect.cpp src/virtual_kinect.cpp src/vkin_offline.cpp)
target_link_libraries( test_vkin_server vtkCommon vtkIO)








