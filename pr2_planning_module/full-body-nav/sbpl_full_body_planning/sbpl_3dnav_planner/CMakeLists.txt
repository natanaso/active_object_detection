cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

#Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE RelWithDebInfo)

# Add a ROS C++ macro to tell SBPL to use ROS print statements
add_definitions(-DROS)

rosbuild_init()

rosbuild_add_boost_directories()

#uncomment if you have defined services
rosbuild_gensrv()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_add_library(aviz src/visualize_arm.cpp)

#rosbuild_add_executable(testDualCollisionSpace src/test_sbpl_dual_collision_space.cpp)
#target_link_libraries(testDualCollisionSpace aviz)
#target_link_libraries(testDualCollisionSpace sbpl_two_arm_planner)

rosbuild_add_library(${PROJECT_NAME} src/sbpl_3dnav_planner.cpp)
target_link_libraries(${PROJECT_NAME} sbpl_full_body_planner aviz pviz)

rosbuild_add_executable(test_get_base_poses src/test_get_base_poses.cpp)

