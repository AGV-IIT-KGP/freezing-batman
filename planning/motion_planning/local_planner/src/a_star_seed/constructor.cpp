//
//  a_star_seed.cpp
//  AStarSeed
//
//  Created by Satya Prakash on 06/09/13.
//  Copyright (c) 2013 Satya Prakash. All rights reserved.
//


#include "a_star_seed/a_star_seed.hpp"

//remove this
#include "ros/package.h"
namespace navigation {
    AStarSeed::AStarSeed(const std::string& seed_file) : SEEDS_FILE(ros::package::getPath("local_planner")+"/seeds/seeds2.txt")
    {
        loadGivenSeeds();
    }
}
