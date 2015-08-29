cmake_minimum_required(VERSION 2.4.6)
project(showpath_debugger)
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
            tf
            message_generation
	    std_msgs	
            dynamic_reconfigure
            pcl_conversions
            rostest
            costmap_2d
            pluginlib
            angles
	    image_transport
	    cv_bridge
	    agv_framework
        )

include_directories(  ${catkin_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/../local_planner/include)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)


catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES compute_cdist
  CATKIN_DEPENDS roscpp rospy cv_bridge image_transport message_runtime
#  DEPENDS assimp Eigen mlpack
)


#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)


add_executable(debugger src/debugger.cpp)
target_link_libraries(debugger ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})
