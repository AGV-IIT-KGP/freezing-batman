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

find_package(MRPT REQUIRED topography)
LINK_DIRECTORIES(${MRPT_DIR}/lib)
LINK_DIRECTORIES(${MRPT_DIR}/lib/otherlibs)
LINK_DIRECTORIES(${MRPT_SOURCE_DIR}/lib/otherlibs)

set(AGV_FRAMEWORK libagv_framework.so)
rosbuild_add_executable(${PROJECT_NAME} src/PosefromTarget.cpp)
target_link_libraries(${PROJECT_NAME} ${AGV_FRAMEWORK} ${MRPT_LIBS})
