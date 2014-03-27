//
//  AStarSeedTester.cpp
//  AStarSeed
//
//  Created by Satya Prakash on 27/03/14.
//  Copyright (c) 2014 Satya Prakash. All rights reserved.
//


#include <iostream>
#include <chrono>

#include "AStarSeed.h"

int main(){
    
    navigation::State botLocation(100,100,90,0),targetLocation(900,900,90,0);
    
    navigation::AStarSeed planner;
    srand((unsigned int)time(NULL));

    
    while (1) {

        planner.mapWithObstacles = 1;
        planner.addObstacles(100+rand()%600, 100+rand()%600, 20+rand()%50);
        planner.addObstacles(100+rand()%600, 100+rand()%600, 20+rand()%50);
        planner.addObstacles(100+rand()%600, 100+rand()%600, 20+rand()%50);
        planner.addObstacles(100+rand()%600, 100+rand()%600, 20+rand()%50);
        planner.addObstacles(100+rand()%600, 100+rand()%600, 20+rand()%50);

        planner.addObstacles(600, 300, 20+rand()%50);
        
        
        std::chrono::steady_clock::time_point startC=std::chrono::steady_clock::now();
        
        auto path = planner.findPathToTargetWithAstar(botLocation, targetLocation);
        
        std::chrono::steady_clock::time_point endC=std::chrono::steady_clock::now();
        
        planner.showPath(path);
        long long takenTime=std::chrono::duration_cast<std::chrono::microseconds > (endC - startC).count();
        std::cout<<"Taken time: "<<takenTime<<std::endl;
        std::cout<<"FPS : "<<(1000000.0)/takenTime<<std::endl;

    }

}