 //
//  main.cpp
//  GlobalPlanner
//
//  Created by Satya Prakash on 28/01/14.
//  Copyright (c) 2014 Satya Prakash. All rights reserved.
//

#include <iostream>
#include "global_planner/GlobalPlanner.hpp"
#include <tbb/tbb.h>
#include <boost/graph/astar_search.hpp>
#include "ss_planner/SSPlanner.hpp"
#include "a_star_grid/AStarGrid.hpp"
int main(int argc, const char * argv[])
{

    // insert code here...
    std::cout << "Hello, World!\n";
    
    
    // Testing SSPlanner
    
//    navigation::PathPlannerPtr planner = (navigation::PathPlannerPtr) (new navigation::SSPlanner()) ;
//    navigation::State start{500,100,M_PI_2,0};
//    navigation::State goal{500,900,0,0};
//    
//    planner->traversablePath(start, goal);

    
    
    // Testing AStarGrid
    
    while (1) {
        navigation::SdcPlanner planner;
        planner.tester();
    }
    
    return 0;
}

