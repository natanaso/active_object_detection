cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

#####################################################################
#### ACTION DEFINITIONS (need to uncomment rosbuild_genmsg())
#####################################################################
rosbuild_find_ros_package(actionlib_msgs)
include(${actionlib_msgs_PACKAGE_PATH}/cmake/actionbuild.cmake)
genaction()

#####################################################################
#### ROS STANDARD DEFINITIONS
#####################################################################
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
############################################## (END) #################################3

#####################################################################
#### LIBRARY INCLUDES
#####################################################################
find_package(EIGEN REQUIRED)
if (EIGEN_FOUND)
  message(STATUS "EIGEN FOUND")
  include_directories(${EIGEN_INCLUDE_DIRS})
  link_directories(${EIGEN_LIBRARY_DIRS})
  add_definitions(${EIGEN_DEFINITIONS})
else()
  message(STATUS "ERROR: EIGEN NOT FOUND")
endif()

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

# thread signals system filesystem program_options regex
rosbuild_add_boost_directories()

#####################################################################
#### LIBRARY BUILDS
#####################################################################

#####################################################################
#### EXECUTABLE BUILDS
#####################################################################
#rosbuild_add_executable(ss_offline src/ss_offline.cpp)
#rosbuild_add_executable(ss_offline_new src/ss_offline_new.cpp)

rosbuild_add_executable(run_ss_decider_offline src/run_ss_decider_offline.cpp src/ss_decider.cpp)
rosbuild_add_executable(run_ss_decider_online src/run_ss_decider_online.cpp src/ss_decider.cpp)

rosbuild_add_executable(ss_arbiter src/ss_arbiter.cpp src/ss_decider.cpp)

rosbuild_add_executable(ss_fastarb src/ss_fastarb.cpp src/ss_decider.cpp)
target_link_libraries( ss_fastarb vtkCommon vtkIO )
rosbuild_link_boost( ss_fastarb system filesystem )

#rosbuild_add_executable(test1 src/test1.cpp)
#rosbuild_link_boost(test1 smart_ptr)
#rosbuild_link_boost(ss_decider thread signals system)





