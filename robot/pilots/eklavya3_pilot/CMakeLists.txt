cmake_minimum_required(VERSION 2.4.6)
project(eklavya3_pilot)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)



find_package(catkin REQUIRED
        COMPONENTS
            cmake_modules
            roscpp
            tf
            message_generation
            dynamic_reconfigure
            pcl_conversions
            rostest
            costmap_2d
            pluginlib
            angles
        )

include_directories(  ${catkin_INCLUDE_DIRS})

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)


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
add_executable(${PROJECT_NAME} src/eklavya3_pilot.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${PCL_LIBRARIES})

#target_link_libraries(example ${PROJECT_NAME})
