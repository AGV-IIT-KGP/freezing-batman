//
//  a_star_seed.cpp
//  AStarSeed
//
//  Created by Satya Prakash on 06/09/13.
//  Copyright (c) 2013 Satya Prakash. All rights reserved.
//


#include "a_star_seed/a_star_seed.hpp"
namespace navigation {
 /*TO DO :
 isWalkable ()
 output, input should be correct */
    
    bool AStarSeed::isWalkableWithSeeds(StateOfCar const& startState, StateOfCar const& targetState) {
        
        bool NoObstacle = true;

        for (std::vector<State>::iterator stateIt = givenSeeds[targetState.seedTaken()].intermediatePoints.begin(); stateIt != givenSeeds[targetState.seedTaken()].intermediatePoints.end(); ++stateIt) {
            
            const State state= *stateIt;
            
            double alpha = startState.theta();
            
            int x = state.x();
            int y = state.y();
            
            int intermediateXcordinate = (int) (x * sin(alpha * (CV_PI / 180)) + y * cos(alpha * (CV_PI / 180)) + startState.x());
            
            int intermediateYcordinate = (int) (-x * cos(alpha * (CV_PI / 180)) + y * sin(alpha * (CV_PI / 180)) + startState.y());
            
            if (((intermediateXcordinate >= 0) && (intermediateXcordinate < MAP_MAX)) && (( intermediateYcordinate >= 0) && (intermediateYcordinate < MAP_MAX))) {

                fusionMap.at<uchar>(fusionMap.rows - intermediateYcordinate -1, intermediateXcordinate) < 128 ? NoObstacle *= 1 : NoObstacle *= 0;
    

            } else {
                return false;
            }
        }
        
        if(fusionMap.at<uchar>(fusionMap.rows - targetState.y() -1, targetState.x()) != 0)
            return false;
        return NoObstacle == true;
    }
}
