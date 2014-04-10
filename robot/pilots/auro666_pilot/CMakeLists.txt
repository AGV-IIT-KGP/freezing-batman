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
set(AGV_FRAMEWORK libagv_framework.so)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
endif()

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

# TODO: Move all of gazebo plugins and its config files into a separate package and depend on it
rosbuild_add_library(plugin src/simulation/AuRo666Plugin.cpp)
target_link_libraries(plugin ${GAZEBO_libraries})

rosbuild_add_executable(controller
                        src/control/controller.cpp 
                        src/control/SteeringController.cpp 
                        src/control/CruiseController.cpp)
target_link_libraries(controller ${AGV_FRAMEWORK})
rosbuild_add_executable(controller_unit_tester src/control/UnitTester.cpp)
rosbuild_add_executable(controller_debugger src/control/Debugger.cpp)

rosbuild_add_executable(roboteq_server 
                        src/interface/roboteq/RoboteqServer.cpp 
                        src/interface/roboteq/RoboteqDevice.cpp)
rosbuild_add_executable(atmega_server 
                        src/interface/atmega/AtmegaServer.cpp
                        src/interface/atmega/AtmegaDevice.cpp
                        src/interface/atmega/Serial.cpp)
rosbuild_add_executable(vehicle_interface src/interface/VehicleInterface.cpp)
rosbuild_add_executable(simple_controller src/control/SimpleController.cpp)
