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

include_directories(${PROJECT_SOURCE_DIR}/include)
#configure_file(config.h.in ${PROJECT_SOURCE_DIR}/include/config.h)

#############################################################################################################################################
#SVM start

set(svm_SRC ${PROJECT_SOURCE_DIR}/src/learner/libsvm/svmWrapper.cpp ${PROJECT_SOURCE_DIR}/src/learner/libsvm/svm.cpp)
set_source_files_properties(${svm_SRC} PROPERTIES COMPILE_FLAGS "-O3 -fPIC" )
add_library(svm ${svm_SRC})
#set(lib_SVM ${PROJECT_SOURCE_DIR}/libsvmWrapper.so ${PROJECT_SOURCE_DIR}/libsvm.so)
set(lib_SVM ${PROJECT_SOURCE_DIR}/lib/libsvm.a)

rosbuild_add_executable(generateSamples src/learner/generateSamples.cpp)

rosbuild_add_executable(svmTrain src/learner/svmTrain.cpp)
target_link_libraries(svmTrain ${lib_SVM})

#SVM end
#############################################################################################################################################

rosbuild_add_library(laneDetector
                     src/detector/getLaneBinary.cpp
                     src/detector/grassRemoval.cpp
                     src/detector/inversePerspectiveTransform.cpp
                     src/detector/laneDetector.cpp
                     src/detector/obstacleRemoval.cpp
                     src/detector/preprocessing.cpp)
target_link_libraries(laneDetector svm)

rosbuild_add_executable(lane_detector src/laneDetectorNode.cpp)
target_link_libraries(lane_detector laneDetector libagv_framework.so m)
