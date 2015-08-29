cmake_minimum_required(VERSION 2.4.6)
project(waypoint_navigator)
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
	    std_msgs
	    geometry_msgs
	    sensor_msgs
       )
find_package(OpenCV REQUIRED)
include_directories(  ${catkin_INCLUDE_DIRS}  ${OpenCV_INCLUDE_DIRS} )
include_directories(SYSTEM ${OpenCV_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR}/include)

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
add_executable(waypoint_navigator  
                        src/main.cpp
                        src/waypoint_navigator.cpp)
target_link_libraries(waypoint_navigator ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})
