cmake_minimum_required(VERSION 2.4.6)
project(lane_detector)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  agv_framework
  cv_bridge
  #opencv2
  image_transport
  roslib  
  message_filters 
  std_msgs 
  geometry_msgs 
  nav_msgs sensor_msgs 
  cmake_modules  
  message_generation
)
find_package(OpenCV REQUIRED)
include_directories(  ${catkin_INCLUDE_DIRS}  ${OpenCV_INCLUDE_DIRS} )
include_directories(SYSTEM ${OpenCV_INCLUDE_DIRS})
find_package(PCL 1.3 REQUIRED COMPONENTS common io)
include_directories(  ${catkin_INCLUDE_DIRS}  ${OpenCV_INCLUDE_DIRS} )
include_directories(SYSTEM ${OpenCV_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)
generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)
catkin_package(
  INCLUDE_DIRS include
  #INCLUDE_DIRS include ${gps_common_INCLUDE_DIRS}}
  #LIBRARIES filter_base ekf
#  LIBRARIES compute_cdist
  CATKIN_DEPENDS roscpp rospy cv_bridge image_transport message_filters std_msgs geometry_msgs nav_msgs  cmake_modules message_runtime
  DEPENDS Eigen
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

include_directories(${PROJECT_SOURCE_DIR}/include)
#configure_file(config.h.in ${PROJECT_SOURCE_DIR}/include/config.h)

#############################################################################################################################################
#SVM start

set(svm_SRC ${PROJECT_SOURCE_DIR}/src/learner/libsvm/svmWrapper.cpp ${PROJECT_SOURCE_DIR}/src/learner/libsvm/svm.cpp)
set_source_files_properties(${svm_SRC} PROPERTIES COMPILE_FLAGS "-O3 -fPIC" )
add_library(svm ${svm_SRC})
#set(lib_SVM ${PROJECT_SOURCE_DIR}/libsvmWrapper.so ${PROJECT_SOURCE_DIR}/libsvm.so)
#set(lib_SVM ${PROJECT_SOURCE_DIR}/lib/libsvm.a)
add_executable(generateSamples src/learner/generateSamples.cpp)
target_link_libraries(generateSamples ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})
add_executable(svmTrain src/learner/svmTrain.cpp ${PROJECT_SOURCE_DIR}/src/learner/libsvm/svmWrapper.cpp ${PROJECT_SOURCE_DIR}/src/learner/libsvm/svm.cpp)
target_link_libraries(svmTrain svm ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})

#SVM end
#############################################################################################################################################

add_library(laneDetector
                     src/detector/getLaneBinary.cpp
                     src/detector/grassRemoval.cpp
                     src/detector/inversePerspectiveTransform.cpp
                     src/detector/laneDetector.cpp
                     src/detector/obstacleRemoval.cpp
                     src/detector/preprocessing.cpp
		     src/detector/shadowRemoval.cpp
		     src/detector/cloudpoint.cpp)
target_link_libraries(laneDetector svm ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})

add_executable(lane_detector src/laneDetectorNode.cpp)
target_link_libraries(lane_detector laneDetector agv_framework m  ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES})

add_executable(mapping src/mapping.cpp)
target_link_libraries(mapping laneDetector agv_framework m  ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES})


#install(TARGETS mapping
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
  PATTERN ".svn" EXCLUDE)