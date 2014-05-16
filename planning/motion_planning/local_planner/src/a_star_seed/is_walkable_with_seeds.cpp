//
//  a_star_seed.cpp
//  AStarSeed
//
//  Created by Satya Prakash on 06/09/13.
//  Copyright (c) 2013 Satya Prakash. All rights reserved.
//

const int MAP_MAX_ = 1000;

#include "a_star_seed/a_star_seed.hpp"
namespace navigation {

    /*TO DO :
    isWalkable ()
    output, input should be correct */

    bool AStarSeed::isWalkableWithSeeds(StateOfCar const& startState, StateOfCar const& targetState, int MAP_MAX_COLS, int MAP_MAX_ROWS) {

        bool NoObstacle = true;

        for (std::vector<State>::iterator stateIt = givenSeeds[targetState.seedTaken()].intermediatePoints.begin(); stateIt != givenSeeds[targetState.seedTaken()].intermediatePoints.end(); ++stateIt) {

            const State state = *stateIt;

            double alpha = startState.theta();

            int x = state.x();
            int y = state.y();

            int intermediateXcordinate = (int) (x * sin(alpha * (CV_PI / 180)) + y * cos(alpha * (CV_PI / 180)) + startState.x());

            int intermediateYcordinate = (int) (-x * cos(alpha * (CV_PI / 180)) + y * sin(alpha * (CV_PI / 180)) + startState.y());

            if (((intermediateXcordinate >= 0) && (intermediateXcordinate < MAP_MAX_COLS)) && ((intermediateYcordinate >= 0) && (intermediateYcordinate < MAP_MAX_ROWS))) {

                fusionMap.at<uchar>(fusionMap.rows - intermediateYcordinate - 1, intermediateXcordinate) < 250 ? NoObstacle *= 1 : NoObstacle *= 0;


            } else {
                return false;
            }
        }

        if (fusionMap.at<uchar>(fusionMap.rows - targetState.y() - 1, targetState.x()) >= 250)
            return false;
        return NoObstacle == true;
    }
    bool quickReflex::isWalkableWithSeeds(State const& startState, State const& targetState, Seed targetSeed) {
        
        bool NoObstacle = true;

        for (std::vector<State>::iterator stateIt = targetSeed.intermediatePoints.begin(); stateIt != targetSeed.intermediatePoints.end(); ++stateIt) {

            int intermediateXcordinate = stateIt->x() + startState.x();
            int intermediateYcordinate = stateIt->y() + startState.y();
            
            if (((intermediateXcordinate >= 0) && (intermediateXcordinate < MAP_MAX_)) && (( intermediateYcordinate >= 0) && (intermediateYcordinate < MAP_MAX_))) {

                fusionMap.at<uchar>(fusionMap.rows - intermediateYcordinate -1, intermediateXcordinate) < 250 ? NoObstacle *= 1 : NoObstacle *= 0;
    
            } 
            else {
                return false;
            }
        }
        
        if(fusionMap.at<uchar>(fusionMap.rows - targetSeed.finalState.y() -1, targetSeed.finalState.x()) >= 250)
            return false;
        return NoObstacle == true;
    }
}
