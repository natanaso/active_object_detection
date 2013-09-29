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

#rosbuild_add_compile_flags(-g)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_add_library(sbpl_arm_planner 
                     src/discrete_space_information/robarm3d/environment_robarm3d.cpp
                     src/bfs_3d.cpp
                     src/bresenham.cpp
                     src/occupancy_grid.cpp
                     src/sbpl_arm_model.cpp
                     src/sbpl_collision_space.cpp
                     src/sbpl_arm_planner_params.cpp
                     src/pr2/sbpl_math.cpp
                     src/pr2/pr2_workspace.cpp
                     src/pr2/orientation_solver.cpp
                     src/sbpl_geometry_utils.cpp)

rosbuild_add_executable(planPath src/test/plan_path.cpp)
rosbuild_add_executable(planPathTest src/test/plan_path_test.cpp)

target_link_libraries(planPath sbpl_arm_planner)
target_link_libraries(planPathTest sbpl_arm_planner)

#rosbuild_add_executable(testBFS src/test/run_bfs_3d.cpp src/bfs_3d.cpp)

