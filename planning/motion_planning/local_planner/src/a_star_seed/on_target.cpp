//
//  a_star_seed.cpp
//  AStarSeed
//
//  Created by Satya Prakash on 06/09/13.
//  Copyright (c) 2013 Satya Prakash. All rights reserved.
//


#include "a_star_seed/a_star_seed.hpp"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

//remove this
#include "ros/package.h"
namespace navigation {
    bool AStarSeed::onTarget(StateOfCar const& currentState, const StateOfCar& targetState) {
        
        for (unsigned int i = 0; i < givenSeeds.size(); i++) {
            for (unsigned int j = 0; j < givenSeeds[i].intermediatePoints.size(); j++) {
                const double sx = givenSeeds[i].intermediatePoints[j].x();
                const double sy = givenSeeds[i].finalState.x();
                //double sz = givenSeeds[i].destination.getXcordinate();
                
                const int x ((int) (currentState.x() + sx * sin(currentState.theta() * (CV_PI / 180)) + sy * cos(currentState.theta() * (CV_PI / 180))));
                const int y ((int) (currentState.y() -sx * cos(currentState.theta() * (CV_PI / 180)) + sy * sin(currentState.theta() * (CV_PI / 180))));
                
                StateOfCar temp(State(x,y,0,0));

                if (temp.isCloseTo(targetState)) {
                    return true;
                }
            }
        }
        return false;
    }
}
