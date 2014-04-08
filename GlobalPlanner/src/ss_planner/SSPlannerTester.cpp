 //
//  main.cpp
//  GlobalPlanner
//
//  Created by Satya Prakash on 28/01/14.
//  Copyright (c) 2014 Satya Prakash. All rights reserved.
//

#include <iostream>
#include <algorithm>
#include <chrono>
#include <vector>
#include <ctime>
#include "State.hpp"
#include "SSPlanner.hpp"



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
        
        navigation::State start{rand()%IMG_WIDTH, rand()%IMG_HEIGHT, M_PI_2, 0}, goal{rand()%IMG_WIDTH, rand()%IMG_HEIGHT, M_PI_2, 0};
        
        std::cout<<"start "<<start.x()<<", "<<start.y()<<" : goal "<<goal.x()<<", "<<goal.y()<<std::endl;
        srand((unsigned int)(time(NULL)));

        std::chrono::steady_clock::time_point startC=std::chrono::steady_clock::now();
        
        auto path = planner.traversablePath( start, goal);

        std::chrono::steady_clock::time_point endC=std::chrono::steady_clock::now();
        long long takenTime=std::chrono::duration_cast<std::chrono::microseconds > (endC - startC).count();
        std::cout<<"Taken time: "<<takenTime<<std::endl;
        std::cout<<"FPS : "<<(1000000.0)/takenTime<<std::endl;

		planner.showPath(path);
    }
    
    return 0;
}

