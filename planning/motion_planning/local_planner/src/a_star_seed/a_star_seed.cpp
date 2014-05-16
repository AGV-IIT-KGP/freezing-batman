//
//  a_star_seed.cpp
//  AStarSeed
//
//  Created by Satya Prakash on 06/09/13.
//  Copyright (c) 2013 Satya Prakash. All rights reserved.
//


#include "a_star_seed/a_star_seed.hpp"
static const int LOOP_RATE = 10;

namespace navigation {
    const int AStarSeed::MAX_ITERATIONS = 10000;

    void AStarSeed::publishStatus(ros::NodeHandle& nh, int status) {
        ros::Publisher status_pub = nh.advertise<std_msgs::String>("localplanner/status", 1000);
        ros::Rate loop_rate(LOOP_RATE);
        std_msgs::String msg;

        std::stringstream ss;
        if (status == 0) {
            ss << "NO PATH FOUND";
        } else if (status == 1) {
            ss << "BOT ON TARGET";
        } else if (status == 2) {
            ss << "PATH FOUND";
        } else {
            ss << "OVERFLOW : OPEN LIST SIZE " << status;
        }

        msg.data = ss.str();

        ROS_INFO("%s", msg.data.c_str());

        status_pub.publish(msg);

        ros::spinOnce();

        loop_rate.sleep();
    }

    std::pair <std::vector<StateOfCar>, Seed> AStarSeed::findPathToTargetWithAstar(const cv::Mat& img, const State& start, const State& goal, const int distance_transform, const int debug_current_state) {
        // USE : for garanteed termination of planner
        int no_of_iterations = 0;

        fusionMap = img;
        MAP_MAX_ROWS = img.rows;
        MAP_MAX_COLS = img.cols;
        if (distance_transform == 1) {
            distanceTransform();
        }
        if (debug_current_state) {
            image = fusionMap.clone();
        }
        StateOfCar startState(start), targetState(goal);

        std::map<StateOfCar, open_map_element> openMap;

        std::map<StateOfCar, StateOfCar, comparatorMapState> came_from;

        SS::PriorityQueue<StateOfCar> openSet;

        openSet.push(startState);

        if (startState.isCloseTo(targetState)) {
            publishStatus(nh, 1);
            return std::make_pair(std::vector<StateOfCar>(), Seed());
        } else if (isOnTheObstacle(startState)) {
            std::cout << "Bot is on the Obstacle Map \n";
            return std::make_pair(std::vector<StateOfCar>(), Seed());
        } else if (isOnTheObstacle(targetState)) {
            std::cout << "Target is on the Obstacle Map \n";
            return std::make_pair(std::vector<StateOfCar>(), Seed());
        }

        while (!openSet.empty() && ros::ok()) {

            if (no_of_iterations > MAX_ITERATIONS) {
                publishStatus(nh, openSet.size());
                return std::make_pair(std::vector<StateOfCar>(), Seed());
            }

            StateOfCar currentState = openSet.top();

            if (openMap.find(currentState) != openMap.end() && openMap[currentState].membership == CLOSED) {
                openSet.pop();
            }

            currentState = openSet.top();
            if (debug_current_state) {
                std::cout << "current x : " << currentState.x() << " current y : " << currentState.y() << std::endl;

                plotPointInMap(currentState);
                cv::imshow("[PLANNER] Map", image);
                cvWaitKey(0);
            }
            // TODO : use closeTo instead of onTarget
            if (currentState.isCloseTo(targetState)) {
                //std::cout << "openSet size : " << openSet.size() << "\n";
                //std::cout<<"Target Reached"<<std::endl;
                publishStatus(nh, 2);
                return reconstructPath(currentState, came_from);
            }
            openSet.pop();
            openMap[currentState].membership = UNASSIGNED;
            openMap[currentState].cost = -currentState.gCost();
            openMap[currentState].membership = CLOSED;

            std::vector<StateOfCar> neighborsOfCurrentState = neighborNodesWithSeeds(currentState);

            for (unsigned int iterator = 0; iterator < neighborsOfCurrentState.size(); iterator++) {
                StateOfCar neighbor = neighborsOfCurrentState[iterator];

                double tentativeGCostAlongFollowedPath = neighbor.gCost() + currentState.gCost();
                double admissible = neighbor.distanceTo(targetState);
                double consistent = admissible;
                double intensity = fusionMap.at<uchar>(neighbor.y(), neighbor.x());

                double consistentAndIntensity = (neighbor.hCost() * neighbor.hCost() + 2 + intensity * intensity) / (neighbor.hCost() + intensity + 2);
                double gCostAndIntensity = (tentativeGCostAlongFollowedPath * tentativeGCostAlongFollowedPath + intensity * intensity + 2) / (tentativeGCostAlongFollowedPath + intensity);

                if (!((openMap.find(neighbor) != openMap.end()) &&
                        (openMap[neighbor].membership == OPEN))) {
                    came_from[neighbor] = currentState;
                    neighbor.hCost(consistent);
                    if (distance_transform == 1)
                        neighbor.gCost(gCostAndIntensity);
                    else
                        neighbor.gCost(tentativeGCostAlongFollowedPath);
                    neighbor.updateTotalCost();

                    openSet.push(neighbor);
                    openMap[neighbor].membership = OPEN;
                    openMap[neighbor].cost = neighbor.gCost();
                }
            }
            no_of_iterations++;
        }
        publishStatus(nh, 0);
        return std::make_pair(std::vector<StateOfCar>(), Seed());
    }
}
