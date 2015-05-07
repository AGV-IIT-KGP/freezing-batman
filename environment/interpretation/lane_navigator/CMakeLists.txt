cmake_minimum_required(VERSION 2.4.6)
project(lane_navigator)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  cv_bridge
  image_transport
  roslib
)

find_package(OpenCV REQUIRED)

include_directories(  ${catkin_INCLUDE_DIRS}  ${OpenCV_INCLUDE_DIRS} )
include_directories(SYSTEM ${OpenCV_INCLUDE_DIRS})
# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

#rosbuild_init()


#catkin_package(
# INCLUDE_DIRS include
#  LIBRARIES compute_cdist
#  CATKIN_DEPENDS roscpp rospy cv_bridge image_transport std_msgs sensor_msgs roslib
#  DEPENDS assimp Eigen mlpack
#)

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
add_executable(lane_navigator src/lane_navigator.cpp)
target_link_libraries(lane_navigator ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})
