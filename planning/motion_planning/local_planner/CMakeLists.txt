cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
include(${PROJECT_SOURCE_DIR}/include)
#add_definitions(-std=c++0x)
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
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
rosbuild_add_executable(${PROJECT_NAME} 
                        src/main.cpp 
                        src/local_planner.cpp 
                        src/a_star_seed/a_star_seed.cpp 
                        src/a_star_seed/state.cpp 
                        src/a_star_seed/state_of_car.cpp 
                        src/a_star_seed/seed.cpp 
                        src/a_star_seed/ss_priority_queue.cpp 
                        src/a_star_seed/add_obstacles.cpp 
                        src/a_star_seed/constructor.cpp 
                        src/a_star_seed/distance_transform.cpp 
                        src/a_star_seed/is_on_the_obstacles.cpp 
                        src/a_star_seed/is_walkable_with_seeds.cpp 
                        src/a_star_seed/load_given_seeds.cpp 
                        src/a_star_seed/neighbor_nodes_with_seeds.cpp 
                        src/a_star_seed/on_target.cpp 
                        src/a_star_seed/plot_point_in_map.cpp 
                        src/a_star_seed/reconstruct_path.cpp 
                        src/a_star_seed/quick_reflex.cpp 
                        src/a_star_seed/show_path.cpp)
target_link_libraries(${PROJECT_NAME} ${AGV_FRAMEWORK})

rosbuild_add_executable(local_planner_tester src/tester.cpp)

rosbuild_add_executable(seed_generator src/a_star_seed/generator.cpp)