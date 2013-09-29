cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
############################################## (END) #################################3

#####################################################################
#### LIBRARY INCLUDES
#####################################################################
rosbuild_add_boost_directories()

find_package(EIGEN REQUIRED)
if (EIGEN_FOUND)
  message(STATUS "EIGEN FOUND")
  include_directories(${EIGEN_INCLUDE_DIRS})
  link_directories(${EIGEN_LIBRARY_DIRS})
  add_definitions(${EIGEN_DEFINITIONS})
else()
  message(STATUS "ERROR: EIGEN NOT FOUND")
endif()

#find_package(PCL 1.6 REQUIRED)
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
rosbuild_add_library(misc_utils src/io_utils.cpp src/pcd_utils.cpp src/misc.cpp)

# Requires pcl ros
rosbuild_add_library(ood_vis src/ood_visualization.cpp)
#####################################################################
#### EXECUTABLE BUILDS
#####################################################################
#rosbuild_add_executable(test_dd_sampler src/test_dd_sampler.cpp)
#rosbuild_add_compile_flags(test_dd_sampler -std=c++0x)

rosbuild_add_executable(test_boost_dd_sampler src/test_boost_dd_sampler.cpp)
rosbuild_link_boost( test_boost_dd_sampler random )

