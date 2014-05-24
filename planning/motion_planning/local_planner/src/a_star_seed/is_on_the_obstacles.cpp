//
//  a_star_seed.cpp
//  AStarSeed
//
//  Created by Satya Prakash on 06/09/13.
//  Copyright (c) 2013 Satya Prakash. All rights reserved.
//

#include <a_star_seed/a_star_seed.hpp>

namespace navigation {

    bool AStarSeed::isOnTheObstacle(const State& state) {
        if (fusion_map.rows - state.y() - 1 < 0 || fusion_map.rows - state.y() - 1 >= fusion_map.rows) {
            return 0;
        }
        if (state.x() < 0 || state.x() >= fusion_map.cols) {
            return 0;
        }
        return fusion_map.at<uchar>(fusion_map.rows - state.y() - 1, state.x()) >= 225;
    }

    bool quickReflex::isOnTheObstacle(const State& state) {
        if (fusion_map.rows - state.y() - 1 < 0 || fusion_map.rows - state.y() - 1 >= fusion_map.rows) {
            return 0;
        }
        if (state.x() < 0 || state.x() >= fusion_map.cols) {
            return 0;
        }
        return fusion_map.at<uchar>(fusion_map.rows - state.y() - 1, state.x()) >= 225;
    }
}
