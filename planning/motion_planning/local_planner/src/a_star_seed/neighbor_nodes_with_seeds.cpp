//
//  a_star_seed.cpp
//  AStarSeed
//
//  Created by Satya Prakash on 06/09/13.
//  Copyright (c) 2013 Satya Prakash. All rights reserved.
//


#include "a_star_seed/a_star_seed.hpp"

namespace navigation {

    std::vector<StateOfCar> AStarSeed::neighborNodesWithSeeds(StateOfCar const& currentState) {
        std::vector<StateOfCar> neighbours;

        for (int i = 0; i < givenSeeds.size(); i++) {
            const double deltaX = givenSeeds[i].final_state.x();
            const double deltaY = givenSeeds[i].final_state.y();
            const double deltaZ = givenSeeds[i].final_state.theta();

            const int x = (int) (currentState.x() + deltaX * sin(currentState.theta() * (CV_PI / 180)) + deltaY * cos(currentState.theta() * (CV_PI / 180)));
            const int y = (int) (currentState.y() - deltaX * cos(currentState.theta() * (CV_PI / 180)) + deltaY * sin(currentState.theta() * (CV_PI / 180)));

            const double theta = (int) (deltaZ - (90 - currentState.theta()));

            const StateOfCar neighbour(x, y, theta, 0, givenSeeds[i].costOfseed, 0, i);


            if (!isWalkableWithSeeds(currentState, neighbour, map_max_cols, map_max_rows)) {
                continue;
            }
            neighbours.push_back(neighbour);
        }
        return neighbours;
    }
    std::vector<Seed> quickReflex::neighborNodesWithSeeds(const State&  start,const State&  goal) {
        std::vector<Seed> neighbours;
        
        for ( int i = 0; i < givenSeeds.size(); i++) {
            
            if ( isWalkableWithSeeds(start, goal, givenSeeds[i])) {
                neighbours.push_back(givenSeeds[i]);
            }

        }
        
        return neighbours;
    }
}
