cmake_minimum_required(VERSION 2.4.6)
project(logitech_camera)
find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	cv_bridge
	agv_framework
	image_transport
)
find_package(OpenCV REQUIRED)
#if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
#    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -W -Wall -g -O3 -std=c++0x")
#endif()
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)
include_directories(  ${catkin_INCLUDE_DIRS}  ${OpenCV_INCLUDE_DIRS} )
include_directories(SYSTEM ${OpenCV_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES compute_cdist
  CATKIN_DEPENDS roscpp rospy cv_bridge image_transport
#  DEPENDS assimp Eigen mlpack
)
#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
set(AGV_FRAMEWORK libagv_framework.so m)
include_directories(${PROJECT_SOURCE_DIR}/include)
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
add_executable(${PROJECT_NAME} src/LogitechCamera.cpp)
#target_link_libraries(example ${PROJECT_NAME})
target_link_libraries(${PROJECT_NAME} agv_framework ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})
