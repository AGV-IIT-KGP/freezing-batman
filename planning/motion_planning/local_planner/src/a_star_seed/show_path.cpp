//
//  a_star_seed.cpp
//  AStarSeed
//
//  Created by Satya Prakash on 06/09/13.
//  Copyright (c) 2013 Satya Prakash. All rights reserved.
//


#include "a_star_seed/a_star_seed.hpp"
namespace navigation {
    void AStarSeed::showPath(std::vector<StateOfCar>& path,const State&  startState,const State&  targetState) {
        

        cv::circle(fusionMap, cvPoint(targetState.x(),fusionMap.rows-1-targetState.y()), 5, cvScalar(128),-1);
        cv::line(fusionMap, cvPoint(targetState.x(),fusionMap.rows-1-targetState.y()), cvPoint(targetState.x()+15*cos((targetState.theta()*M_PI)/180),fusionMap.rows-1-targetState.y()-15*sin((targetState.theta()*M_PI)/180)),cvScalar(128),1,8,0);
        cv::circle(fusionMap, cvPoint(startState.x(),fusionMap.rows-1-startState.y()), 5, cvScalar(128),-1);
        cv::line(fusionMap, cvPoint(startState.x(),fusionMap.rows-1-startState.y()), cvPoint(startState.x()+15*cos((startState.theta()*M_PI)/180),fusionMap.rows-1-startState.y()-15*sin((startState.theta()*M_PI)/180)),cvScalar(128),1,8,0);
        std::cout<<"Showing A Path\n";
        
        if(!DEBUG)
            image = fusionMap.clone();

        std::cout<<"Path size in showPath is : " <<(int)path.size()<<" \n";
        for (std::vector<StateOfCar>::iterator stateIt = path.begin(); stateIt != path.end() ; ++stateIt) {
            const State state = *stateIt;
            plotPointInMap(state);
        }

        cv::imshow("[PLANNER] Map", image);
        cvWaitKey(1);
    }
}
