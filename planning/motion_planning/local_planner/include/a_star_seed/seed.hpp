//
//  seed.hpp
//  AStarSeed
//
//  Created by Satya Prakash on 28/01/14.
//  Copyright (c) 2014 Satya Prakash. All rights reserved.
//

#ifndef __AStarSeed__Seed__
#define __AStarSeed__Seed__

#include <iostream>
#include <vector>
#include "state.hpp"

namespace navigation {

    class Seed {
    public:
        State final_state;
        double obstacleCostOfSeed, targetCostOfSeed, velocityRatio, leftVelocity, rightVelocity;
        std::vector<State> intermediatePoints;

        Seed() {
        }

        Seed(const Seed& other) : final_state(other.final_state), obstacleCostOfSeed(other.obstacleCostOfSeed), targetCostOfSeed(other.targetCostOfSeed), velocityRatio(other.velocityRatio), leftVelocity(other.leftVelocity), rightVelocity(other.rightVelocity), intermediatePoints(other.intermediatePoints) {
        }

    };


}
#endif /* defined(__AStarSeed__Seed__) */
