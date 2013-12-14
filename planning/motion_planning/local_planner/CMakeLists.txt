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

set(AGV_FRAMEWORK libagv_core.so)
#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
rosbuild_add_boost_directories()
rosbuild_add_executable(localPlanner src/RoadNavigation.cpp src/Clothoid.cpp src/PathSegment.cpp src/local_planner.cpp src/Trajectory.cpp src/State.cpp)
rosbuild_add_executable(localPlannerTester src/localPlannerTester.cpp src/RoadNavigationTester.cpp)
target_link_libraries(localPlanner libagv_core.so)
rosbuild_link_boost(localPlanner thread system signals)
#target_link_libraries(example ${PROJECT_NAME})
