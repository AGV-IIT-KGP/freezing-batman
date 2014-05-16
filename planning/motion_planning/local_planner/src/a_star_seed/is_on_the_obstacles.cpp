//
//  a_star_seed.cpp
//  AStarSeed
//
//  Created by Satya Prakash on 06/09/13.
//  Copyright (c) 2013 Satya Prakash. All rights reserved.
//


#include "a_star_seed/a_star_seed.hpp"


namespace navigation {
    bool AStarSeed::isOnTheObstacle(const State& state){
        return fusionMap.at<uchar>(fusionMap.rows - state.y() -1, state.x()) >= 225;
    }
    bool quickReflex::isOnTheObstacle(const State& state){
        return fusionMap.at<uchar>(fusionMap.rows - state.y() -1, state.x()) >= 225;
    }
}
