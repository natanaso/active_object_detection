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
#####################################################################
#### LIBRARY INCLUDES
#####################################################################
# EIGEN
#find_package(Eigen REQUIRED)
#include_directories(${EIGEN_INCLUDE_DIRS})
#add_definitions(${EIGEN_DEFINITIONS})
# VTK
#include_directories(/usr/include/vtk-5.8)
#include_directories(/usr/include/vtk-5.6)

find_package(VTK REQUIRED)
if (VTK_FOUND)
  message(STATUS "VTK FOUND")
  include_directories(${VTK_INCLUDE_DIRS}) 
  link_directories(${VTK_LIBRARY_DIRS})
else()
  message(STATUS "ERROR: VTK NOT FOUND")
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

rosbuild_add_boost_directories()
#rosbuild_link_boost(vtree_user thread signals system filesystem program_options regex)
#####################################################################
#### LIBRARY BUILDS
#####################################################################
#rosbuild_add_library(nbv_tree src/nbv_tree.cpp)
#rosbuild_add_library(nbv_tree_util src/nbv_tree_util.cpp)
#target_link_libraries(nbv_tree nbv_tree_util vtkCommon vtkFiltering vtkRendering)

rosbuild_add_library(vtree_user src/vtree_user.cpp src/tabletop_analyzer.cpp)

#####################################################################
#### EXECUTABLE BUILDS
#####################################################################
rosbuild_add_executable(ply2traindata src/ply2traindata.cpp)
target_link_libraries(ply2traindata vtkCommon vtkIO)
rosbuild_link_boost( ply2traindata system filesystem )

rosbuild_add_executable(build_tree src/build_tree.cpp)
target_link_libraries(build_tree vtree_user)

rosbuild_add_executable(build_omap src/build_omap.cpp)
target_link_libraries(build_omap vtree_user vtkCommon vtkIO)

rosbuild_add_executable(build_omap_occ src/build_omap_occ.cpp)
target_link_libraries(build_omap_occ vtree_user vtkCommon vtkIO)

rosbuild_add_executable(pcd_from_ply src/pcd_from_ply.cpp)
target_link_libraries(pcd_from_ply vtree_user vtkCommon vtkIO)

rosbuild_add_executable(score_pcd src/score_pcd.cpp)
target_link_libraries(score_pcd vtree_user ${PCL_LIBRARIES})

rosbuild_add_executable(compute_confmat src/compute_confmat.cpp) 
target_link_libraries(compute_confmat vtree_user vtkCommon vtkIO)

# temporary
#rosbuild_add_executable( fix_omap src/fix_omap.cpp )
#rosbuild_link_boost( fix_omap system filesystem )

#rosbuild_add_executable( fix_omap_v2 src/fix_omap_v2.cpp )
#rosbuild_link_boost( fix_omap_v2 system filesystem )

#rosbuild_add_executable( tmp_fxr src/tmp_fxr.cpp )
#rosbuild_link_boost( tmp_fxr system filesystem )

rosbuild_add_executable( vpm src/views_to_pcd_model.cpp )

