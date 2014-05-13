//
//  a_star_seed.cpp
//  AStarSeed
//
//  Created by Satya Prakash on 06/09/13.
//  Copyright (c) 2013 Satya Prakash. All rights reserved.
//


#include "a_star_seed/a_star_seed.hpp"

//remove this
#include "ros/ros.h"
namespace navigation {
    const int AStarSeed::MAX_ITERATIONS = 10000;

    std::pair<std::vector<StateOfCar>, Seed> AStarSeed::findPathToTargetWithAstar(const cv::Mat& img,const State&  start,const State&  goal) {
        // USE : for garanteed termination of planner
        int no_of_iterations = 0;

        fusionMap = img;
  
        if(DT==1)
            distanceTransform();
        if (DEBUG)  {
            image = fusionMap.clone();
        }
        StateOfCar startState(start), targetState(goal);

        std::map<StateOfCar, open_map_element> openMap;

        std::map<StateOfCar,StateOfCar, comparatorMapState> came_from;

        SS::PriorityQueue<StateOfCar> openSet;

        openSet.push(startState);

        if (startState.isCloseTo(targetState)) {
            std::cout<<"Bot is On Target"<<std::endl;
            return std::make_pair(std::vector<StateOfCar>(), Seed());
        }
        
        else if(isOnTheObstacle(startState)){
            std::cout<<"Bot is on the Obstacle Map \n";
            return std::make_pair(std::vector<StateOfCar>(), Seed());
        }
        else if(isOnTheObstacle(targetState)){
            std::cout<<"Target is on the Obstacle Map \n";
            return std::make_pair(std::vector<StateOfCar>(), Seed());
        }

        while (!openSet.empty() && ros::ok()) {
            // std::cout<<"openSet size : "<<openSet.size()<<"\n";

            if(no_of_iterations > MAX_ITERATIONS){
                std::cerr<<"Overflow : openlist size : "<<openSet.size()<<"\n";
            return std::make_pair(std::vector<StateOfCar>(), Seed());
            }

            StateOfCar currentState=openSet.top();

            if (openMap.find(currentState)!=openMap.end() && openMap[currentState].membership == CLOSED) {
                openSet.pop();
            }
            
            currentState=openSet.top();

            if (DEBUG)  {
               std::cout<<"current x : "<<currentState.x()<<" current y : "<<currentState.y()<<std::endl;

               plotPointInMap(currentState);
               cv::imshow("[PLANNER] Map", image);
               cvWaitKey(0);
            }
            // TODO : use closeTo instead of onTarget
            if (currentState.isCloseTo(targetState)) {
               std::cout<<"openSet size : "<<openSet.size()<<"\n";
//                std::cout<<"Target Reached"<<std::endl;
                return reconstructPath(currentState, came_from);
            }
            openSet.pop();
            openMap[currentState].membership = UNASSIGNED;
            openMap[currentState].cost=-currentState.gCost(); 
            openMap[currentState].membership=CLOSED;
            
            std::vector<StateOfCar> neighborsOfCurrentState = neighborNodesWithSeeds(currentState);
            
            for (unsigned int iterator=0; iterator < neighborsOfCurrentState.size(); iterator++) {
                StateOfCar neighbor = neighborsOfCurrentState[iterator];

                double tentativeGCostAlongFollowedPath = neighbor.gCost() + currentState.gCost();
                double admissible = neighbor.distanceTo(targetState);
                double consistent = admissible;
                double intensity = fusionMap.at<uchar>(neighbor.y(), neighbor.x());
                
                double consistentAndIntensity = (neighbor.hCost()*neighbor.hCost()+2 + intensity*intensity)/(neighbor.hCost()+intensity+2);

                
                if (!((openMap.find(neighbor) != openMap.end()) &&
                      (openMap[neighbor].membership == OPEN))) {
                    came_from[neighbor] = currentState;
                    neighbor.gCost( tentativeGCostAlongFollowedPath) ;
                    if(DT==1)
                        neighbor.hCost(consistentAndIntensity);
                    else
                        neighbor.hCost( consistent) ;
                    neighbor.updateTotalCost();

                    openSet.push(neighbor);
                    openMap[neighbor].membership = OPEN;
                    openMap[neighbor].cost = neighbor.gCost();
                }
            }
            no_of_iterations++;
        }
        std::cerr<<"NO PATH FOUND"<<std::endl;
            return std::make_pair(std::vector<StateOfCar>(), Seed());
    }
}
