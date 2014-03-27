cmake_minimum_required (VERSION 2.8)
project (GlobalPlanner)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
#SET (CMAKE_C_COMPILER             "/usr/bin/clang")
#SET (CMAKE_CXX_COMPILER             "/usr/bin/clang++")


#TODO : for gcc 4.8.1 only -std flag
ADD_DEFINITIONS(
    -std=c++11 # Or -std=c++0x
    -stdlib=libc++# Other flags
)
find_package(OpenCV REQUIRED )

add_executable(	globalPlannerTester 
	src/global_planner/GlobalPlannerTester.cpp
	src/global_planner/GlobalPlanner.cpp  
	src/a_star_grid/AStarGrid.cpp
	src/utils/State.cpp 
	src/utils/Path.cpp 
	src/utils/PathSegment.cpp)

add_executable(	SSPlannerTester
	src/ss_planner/SSPlannerTester.cpp 
	src/ss_planner/SSPlanner.cpp 
	src/ss_planner/SSPath.cpp 
	src/ss_planner/SSState.cpp 
	src/ss_planner/SSPathSegment.cpp 
	src/ss_planner/SSPriority_queue.cpp
	src/utils/State.cpp 	
	src/utils/Path.cpp 
	src/utils/PathSegment.cpp)

add_executable(aStarSeedTester 
	src/a_star_seed/AStarSeedTester.cpp 
	src/a_star_seed/AStarSeed.cpp 
	src/a_star_seed/StateOfCar.cpp 
	src/a_star_seed/SSPriority_queue.cpp 
	src/a_star_seed/State.cpp)

add_executable(aStarGridTester 
	src/a_star_grid/AStarGridTester.cpp 
	src/a_star_grid/AStarGrid.cpp 
	src/a_star_grid/GridState.cpp 
	src/utils/State.cpp 	
	src/utils/Path.cpp 
	src/utils/PathSegment.cpp)

target_link_libraries( globalPlannerTester ${OpenCV_LIBS} )
target_link_libraries( aStarGridTester ${OpenCV_LIBS} )
target_link_libraries( SSPlannerTester ${OpenCV_LIBS} )
target_link_libraries( aStarSeedTester ${OpenCV_LIBS} )
