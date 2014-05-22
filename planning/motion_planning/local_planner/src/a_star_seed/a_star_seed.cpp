//
//  a_star_seed.cpp
//  AStarSeed
//
//  Created by Satya Prakash on 06/09/13.
//  Copyright (c) 2013 Satya Prakash. All rights reserved.
//

#include <a_star_seed/a_star_seed.hpp>

namespace navigation {

    std::pair <std::vector<StateOfCar>, Seed> AStarSeed::findPathToTarget(const cv::Mat& map, const State& start, const State& goal, const int distance_transform, const int debug_current_state, int& status) {
        // USE : for guaranteed termination of planner
        int no_of_iterations = 0;
        int max_iterations = 10000;

        node_handle.getParam("local_planner/max_iterations", max_iterations);

        fusion_map = map;
        map_max_rows = map.rows;
        map_max_cols = map.cols;

        if (distance_transform == 1) {
            distanceTransform();
        }

        if (debug_current_state) {
            image = fusion_map.clone();
        }
        
        StateOfCar start_state(start), target_state(goal);
        std::map<StateOfCar, open_map_element> open_map;
        std::map<StateOfCar, StateOfCar, comparatorMapState> came_from;
        SS::PriorityQueue<StateOfCar> open_set;

        open_set.push(start_state);

        if (start_state.isCloseTo(target_state)) {
            status = 1;
            return std::make_pair(std::vector<StateOfCar>(), Seed());
        }
        else if (isOnTheObstacle(start_state)) {
            std::cout << "Bot is on the Obstacle Map \n";
            return std::make_pair(std::vector<StateOfCar>(), Seed());
        }
        else if (isOnTheObstacle(target_state)) {
            std::cout << "Target is on the Obstacle Map \n";
            return std::make_pair(std::vector<StateOfCar>(), Seed());
        }

        while (!open_set.empty() && ros::ok()) {
            if (no_of_iterations > max_iterations) {
                status = open_set.size();
                return std::make_pair(std::vector<StateOfCar>(), Seed());
            }

            StateOfCar current_state = open_set.top();

            if (open_map.find(current_state) != open_map.end() && open_map[current_state].membership == CLOSED) {
                open_set.pop();
            }

            current_state = open_set.top();
            if (debug_current_state) {
                std::cout << "current x : " << current_state.x() << " current y : " << current_state.y() << std::endl;

                plotPointInMap(current_state);
                cv::imshow("[PLANNER] Map", image);
                cvWaitKey(0);
            }
            // TODO : use closeTo instead of onTarget
            if (current_state.isCloseTo(target_state)) {
                status = 2;
                return reconstructPath(current_state, came_from);
            }

            open_set.pop();
            open_map[current_state].membership = UNASSIGNED;
            open_map[current_state].cost = -current_state.gCost();
            open_map[current_state].membership = CLOSED;

            std::vector<StateOfCar> neighbors_of_current_state = neighborNodesWithSeeds(current_state);

            for (unsigned int iterator = 0; iterator < neighbors_of_current_state.size(); iterator++) {
                StateOfCar neighbor = neighbors_of_current_state[iterator];

                double tentative_gcost_along_followed_path = neighbor.gCost() + current_state.gCost();
                double admissible = neighbor.distanceTo(target_state);
                double consistent = admissible;
                double intensity = fusion_map.at<uchar>(neighbor.y(), neighbor.x());
                // double consistentAndIntensity = (consistent*consistent + 2 + intensity * intensity) / (consistent + intensity + 2);
                double gcost_and_intensity = (tentative_gcost_along_followed_path * tentative_gcost_along_followed_path + intensity * intensity + 2) / (tentative_gcost_along_followed_path + intensity);

                if (!((open_map.find(neighbor) != open_map.end()) &&
                      (open_map[neighbor].membership == OPEN))) {
                    came_from[neighbor] = current_state;
                    neighbor.hCost(consistent);
                    if (distance_transform == 1) {
                        neighbor.gCost(gcost_and_intensity);
                    } else {
                        neighbor.gCost(tentative_gcost_along_followed_path);
                    }
                    neighbor.updateTotalCost();

                    open_set.push(neighbor);
                    open_map[neighbor].membership = OPEN;
                    open_map[neighbor].cost = neighbor.gCost();
                }
            }
            no_of_iterations++;
        }
        status = 0;
        return std::make_pair(std::vector<StateOfCar>(), Seed());
    }
}
