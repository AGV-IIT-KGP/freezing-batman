cmake_minimum_required(VERSION 2.4.6)
project(dummy_navigator)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
include(${PROJECT_SOURCE_DIR}/include)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

find_package(catkin REQUIRED
        COMPONENTS
            cmake_modules
            roscpp
            message_generation
	    std_msgs	
            pluginlib
            )

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES compute_cdist
  CATKIN_DEPENDS roscpp rospy
#  DEPENDS assimp Eigen mlpack
)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
#set(STATE libState.so)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
add_executable(dummy_navigator src/dummy_navigator.cpp)
target_link_libraries(dummy_navigator ${catkin_LIBRARIES})
#rosbuild_link_boost(Navigator signals)
#target_link_libraries(example ${PROJECT_NAME})
#target_link_libraries(Navigator ${STATE})
