 //
//  main.cpp
//  GlobalPlanner
//
//  Created by Satya Prakash on 28/01/14.
//  Copyright (c) 2014 Satya Prakash. All rights reserved.
//

#include <iostream>
#include "utils/State.hpp"
#include "ss_planner/SSPlanner.hpp"

int main(int argc, const char * argv[])
{

	srand((unsigned int)time(NULL));
    // insert code here...
    std::cout << "Hello, World!\n";
    
    
    // Testing SSPlanner
    
//    navigation::PathPlannerPtr planner = (navigation::PathPlannerPtr) (new navigation::SSPlanner()) ;
//    navigation::State start{500,100,M_PI_2,0};
//    navigation::State goal{500,900,0,0};
//    
//    planner->traversablePath(start, goal);

    
    
    // Testing AStarGrid
    
	navigation::SSPlanner planner;

    while (1) {
        planner.traversablePath(navigation::State(rand()%1000, rand()%1000, 0, 0), navigation::State(rand()%1000, rand()%1000, 0, 0));
		planner.showPath();
    }
    
    return 0;
}

