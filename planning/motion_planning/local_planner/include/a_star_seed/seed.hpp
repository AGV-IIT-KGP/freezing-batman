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
        State finalState;
        double costOfseed, velocityRatio, leftVelocity, rightVelocity;
        std::vector<State> intermediatePoints;

        Seed() {
        }

        Seed(const Seed& that) : finalState(that.finalState), costOfseed(that.costOfseed), velocityRatio(that.velocityRatio), leftVelocity(that.leftVelocity), rightVelocity(that.rightVelocity), intermediatePoints(that.intermediatePoints) {
        }

    };


}
#endif /* defined(__AStarSeed__Seed__) */
