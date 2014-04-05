//
//  Seed.hpp
//  GlobalPlanner
//
//  Created by Satya Prakash on 30/03/14.
//  Copyright (c) 2014 Satya Prakash. All rights reserved.
//

#ifndef __AStarSeed__Seed__
#define __AStarSeed__Seed__

#include <iostream>
#include <vector>
#include "State.hpp"

namespace navigation {
    
    
    class Seed{
    public:
        State finalState;
        double costOfseed,velocityRatio,leftVelocity,rightVelocity;
        std::vector<State> intermediatePoints;
    };
    
    
}
#endif /* defined(__AStarSeed__Seed__) */
