cmake_minimum_required(VERSION 2.4.6)
project(agv_framework)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)
include_directories(
${PROJECT_SOURCE_DIR}/include)

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
add_library(${PROJECT_NAME}
  src/HealthMonitor.cpp
  src/LifeCycle.cpp
  src/environment/Environment.cpp
  src/environment/Interpreter.cpp
  src/environment/Model.cpp
  src/environment/Sensor.cpp
  src/planning/MasterPlanner.cpp
  src/planning/MotionPlanner.cpp
  src/planning/motion_planner/GlobalPlanner.cpp
  src/planning/motion_planner/LocalPlanner.cpp
  src/planning/planner.cpp
  src/planning/motion_planner/SituationalPlanner.cpp
  src/robot/Description.cpp
  src/robot/Interpreter.cpp
  src/robot/Robot.cpp
  src/robot/Sensor.cpp
  src/utils/Pose2D.cpp
  )

catkin_package(  INCLUDE_DIRS include include
#  LIBRARIES compute_cdist
  CATKIN_DEPENDS roscpp 
#  DEPENDS assimp Eigen mlpack
)
#rosbuild_add_library(agv src/Sensor.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#target_link_libraries(example ${PROJECT_NAME})
