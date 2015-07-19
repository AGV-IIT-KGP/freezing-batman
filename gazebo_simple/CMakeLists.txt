cmake_minimum_required(VERSION 2.8.3)
project(gazebo_simple)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roscpp
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES gazebo_simple
   CATKIN_DEPENDS gazebo_ros roscpp
#  DEPENDS system_lib
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(gazebo_simple_node src/turtle_teleop.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(gazebo_simple_node  ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
