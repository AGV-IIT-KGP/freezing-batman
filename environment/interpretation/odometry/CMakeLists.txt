
cmake_minimum_required(version 2.4.6)
if(cmake_compiler_is_gnucc or cmake_compiler_is_gnucxx)
    set(cmake_cxx_flags "${cmake_cxx_flags} -w -wall -g -o3 -std=c++0x")
endif()
include($env{ros_root}/core/rosbuild/rosbuild.cmake)

# set the build type.  options are:
#  coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  debug          : w/ debug symbols, w/o optimization
#  release        : w/o debug symbols, w/ optimization
#  relwithdebinfo : w/ debug symbols, w/ optimization
#  minsizerel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ros_build_type relwithdebinfo)

rosbuild_init()


#set the default path for built executables to the "bin" directory
set(executable_output_path ${project_source_dir}/bin)
#set the default path for built libraries to the "lib" directory
set(library_output_path ${project_source_dir}/lib)
set(agv_framework libagv_framework.so)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${project_name} src/example.cpp)
#target_link_libraries(${project_name} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${project_name} thread)
#rosbuild_add_executable(example examples/example.cpp)
rosbuild_add_executable(${project_name} src/odometry_publisher.cpp src/odometry.cpp)

#target_link_libraries(example ${project_name})
target_link_libraries(${project_name} ${agv_framework})
