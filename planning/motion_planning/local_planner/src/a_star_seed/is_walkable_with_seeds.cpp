//
//  a_star_seed.cpp
//  AStarSeed
//
//  Created by Satya Prakash on 06/09/13.
//  Copyright (c) 2013 Satya Prakash. All rights reserved.
//

#include <a_star_seed/a_star_seed.hpp>

const int map_max_ = 1000;

namespace navigation {

    /*TO DO :
    isWalkable ()
    output, input should be correct */

    bool AStarSeed::isWalkableWithSeeds(StateOfCar const& start_state, StateOfCar const& target_state, int map_max_cols, int map_max_rows) {
        bool no_obstacle = true;

        for (std::vector<State>::iterator state_it = givenSeeds[target_state.seedTaken()].intermediatePoints.begin(); state_it != givenSeeds[target_state.seedTaken()].intermediatePoints.end(); ++state_it) {
            const State state = *state_it;
            double alpha = start_state.theta();

            int x = state.x();
            int y = state.y();

            int intermediate_x = (int) (x * sin(alpha * (CV_PI / 180)) + y * cos(alpha * (CV_PI / 180)) + start_state.x());
            int intermediate_y = (int) (-x * cos(alpha * (CV_PI / 180)) + y * sin(alpha * (CV_PI / 180)) + start_state.y());

            if (((intermediate_x >= 0) && (intermediate_x < map_max_cols)) && ((intermediate_y >= 0) && (intermediate_y < map_max_rows))) {
                fusion_map.at<uchar>(fusion_map.rows - intermediate_y - 1, intermediate_x) < 250 ? no_obstacle *= 1 : no_obstacle *= 0;
            } else {
                return false;
            }
        }
        if (fusion_map.rows - target_state.y() - 1 < 0 || fusion_map.rows - target_state.y() - 1 >= fusion_map.rows) {
            return true;
        }
        if (target_state.x() < 0 || target_state.x() >= fusion_map.cols) {
            return true;
        }
        if (fusion_map.at<uchar>(fusion_map.rows - target_state.y() - 1, target_state.x()) >= 250) {
            return false;
        }

        return no_obstacle == true;
    }

    bool quickReflex::isWalkableWithSeeds(State const& start_state, State const& target_state, Seed target_seed) {
        bool no_obstacle = true;

        for (std::vector<State>::iterator state_it = target_seed.intermediatePoints.begin(); state_it != target_seed.intermediatePoints.end(); ++state_it) {
            int intermediate_x = state_it->x() + start_state.x();
            int intermediate_y = state_it->y() + start_state.y();

            if (((intermediate_x >= 0) && (intermediate_x < map_max_)) && ((intermediate_y >= 0) && (intermediate_y < map_max_))) {
                fusion_map.at<uchar>(fusion_map.rows - intermediate_y - 1, intermediate_x) < 250 ? no_obstacle *= 1 : no_obstacle *= 0;
            } else {
                return false;
            }
        }

        if (fusion_map.at<uchar>(fusion_map.rows - target_seed.final_state.y() - 1, target_seed.final_state.x()) >= 250) {
            return false;
        }

        return no_obstacle == true;
    }
}
