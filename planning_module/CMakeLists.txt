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


# Add the BOOST library
rosbuild_add_boost_directories()
#set(Boost_USE_STATIC_LIBS        ON)
#set(Boost_USE_MULTITHREADED      ON)
#set(Boost_USE_STATIC_RUNTIME    OFF)
#find_package( Boost COMPONENTS filesystem system REQUIRED )
#if(Boost_FOUND)
#        include_directories(${Boost_INCLUDE_DIRS} "./include")
#        add_executable(test_sarsop_user src/test_sarsop_user.cpp src/sarsop_user.cpp)
#        target_link_libraries(test_sarsop_user ${Boost_LIBRARIES})
#endif()

# Add the APPL library
add_custom_target(
   appl
   COMMAND make
   WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/appl-0.95/src
)

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

include_directories(${PROJECT_SOURCE_DIR}/appl-0.95/src/Algorithms)
include_directories(${PROJECT_SOURCE_DIR}/appl-0.95/src/Algorithms/HSVI)
include_directories(${PROJECT_SOURCE_DIR}/appl-0.95/src/Algorithms/SARSOP)
include_directories(${PROJECT_SOURCE_DIR}/appl-0.95/src/Bounds)
include_directories(${PROJECT_SOURCE_DIR}/appl-0.95/src/Core)
include_directories(${PROJECT_SOURCE_DIR}/appl-0.95/src/Evaluator)
include_directories(${PROJECT_SOURCE_DIR}/appl-0.95/src/MathLib)
include_directories(${PROJECT_SOURCE_DIR}/appl-0.95/src/miniposix)
include_directories(${PROJECT_SOURCE_DIR}/appl-0.95/src/Models/MOMDP)
include_directories(${PROJECT_SOURCE_DIR}/appl-0.95/src/Models/MOMDP/CoLoc)
include_directories(${PROJECT_SOURCE_DIR}/appl-0.95/src/OfflineSolver)
include_directories(${PROJECT_SOURCE_DIR}/appl-0.95/src/Parser)
include_directories(${PROJECT_SOURCE_DIR}/appl-0.95/src/Parser/POMDPX)
include_directories(${PROJECT_SOURCE_DIR}/appl-0.95/src/Parser/Cassandra)
include_directories(${PROJECT_SOURCE_DIR}/appl-0.95/src/Parser/Cassandra/include)
include_directories(${PROJECT_SOURCE_DIR}/appl-0.95/src/PolicyGraph)
include_directories(${PROJECT_SOURCE_DIR}/appl-0.95/src/PomdpConvertor)
include_directories(${PROJECT_SOURCE_DIR}/appl-0.95/src/Simulator)
include_directories(${PROJECT_SOURCE_DIR}/appl-0.95/src/MUtils)
rosbuild_add_library(sarsop_user src/sarsop_user.cpp)
target_link_libraries(sarsop_user ${PROJECT_SOURCE_DIR}/appl-0.95/src/libappl.a)
rosbuild_link_boost(sarsop_user filesystem system)
add_dependencies(sarsop_user appl)


rosbuild_add_executable(test_sarsop_user src/test_sarsop_user.cpp)
target_link_libraries(test_sarsop_user sarsop_user)
#rosbuild_link_boost(test_sarsop_user filesystem system)
#target_link_libraries(test_sarsop_user ${Boost_LIBRARIES} sarsop_user)


# Other planning
rosbuild_add_library(ejs_planner src/ejs_planner.cpp)
rosbuild_add_library(gmi_planner src/gmi_planner.cpp)
rosbuild_add_library(cvp_planner src/cvp_planner.cpp)
#rosbuild_add_executable(voi_planner src/voi_planner.cpp)









