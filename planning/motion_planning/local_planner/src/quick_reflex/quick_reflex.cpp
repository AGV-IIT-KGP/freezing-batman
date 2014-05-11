#include "quick_reflex/quick_reflex.hpp"

#include "ros/ros.h"

namespace navigation {

    std::pair<std::vector<State>, Seed> quickReflex::findPathToTarget(const cv::Mat& img,const State&  start,const State&  goal) {

        loadGivenSeeds(start, goal);

        fusionMap = img;
  
        // if(DT==1)
        //     distanceTransform();

        // StateOfCar startState(start), targetState(goal);

        // std::map<StateOfCar, open_map_element> openMap;

        // std::map<StateOfCar,StateOfCar, comparatorMapState> came_from;

        // SS::PriorityQueue<StateOfCar> openSet;

        // openSet.push(startState);

        if (start.isCloseTo(goal)) {
            printf("Bot is On Target\n");
            return std::make_pair(std::vector<State>(), Seed());
        }
        
        else if(isOnTheObstacle(goal)){
            printf("Bot is on the Obstacle Map\n");
            return std::make_pair(std::vector<State>(), Seed());
        }
        else if(isOnTheObstacle(goal)){
            printf("Target is on the Obstacle Map\n");
            return std::make_pair(std::vector<State>(), Seed());
        }

//         while (!openSet.empty() && ros::ok()) {
//             // std::cout<<"openSet size : "<<openSet.size()<<"\n";

//             if(no_of_iterations > MAX_ITERATIONS){
//                 std::cerr<<"Overflow : openlist size : "<<openSet.size()<<"\n";
//             return std::make_pair(std::vector<`>(), Seed());
//             }

//             StateOfCar currentState=openSet.top();

//             if (openMap.find(currentState)!=openMap.end() && openMap[currentState].membership == CLOSED) {
//                 openSet.pop();
//             }
            
//             currentState=openSet.top();

//             if (DEBUG)  {
//                std::cout<<"current x : "<<currentState.x()<<" current y : "<<currentState.y()<<std::endl;

//                plotPointInMap(currentState);
//                cv::imshow("[PLANNER] Map", image);
//                cvWaitKey(0);
//             }
//             // TODO : use closeTo instead of onTarget
//             if (currentState.isCloseTo(targetState)) {
//                std::cout<<"openSet size : "<<openSet.size()<<"\n";
// //                std::cout<<"Target Reached"<<std::endl;
//                 return reconstructPath(currentState, came_from);
//             }
//             openSet.pop();
//             openMap[currentState].membership = UNASSIGNED;
//             openMap[currentState].cost=-currentState.gCost(); 
//             openMap[currentState].membership=CLOSED;
            
//             std::vector<StateOfCar> neighborsOfCurrentState = neighborNodesWithSeeds(currentState);
            
//             for (unsigned int iterator=0; iterator < neighborsOfCurrentState.size(); iterator++) {
//                 StateOfCar neighbor = neighborsOfCurrentState[iterator];

//                 double tentativeGCostAlongFollowedPath = neighbor.gCost() + currentState.gCost();
//                 double admissible = neighbor.distanceTo(targetState);
//                 double consistent = admissible;
//                 double intensity = fusionMap.at<uchar>(neighbor.y(), neighbor.x());
                
//                 double consistentAndIntensity = (neighbor.hCost()*neighbor.hCost()+2 + intensity*intensity)/(neighbor.hCost()+intensity+2);

                
//                 if (!((openMap.find(neighbor) != openMap.end()) &&
//                       (openMap[neighbor].membership == OPEN))) {
//                     came_from[neighbor] = currentState;
//                     neighbor.gCost( tentativeGCostAlongFollowedPath) ;
//                     if(DT==1)
//                         neighbor.hCost(consistentAndIntensity);
//                     else
//                         neighbor.hCost( consistent) ;
//                     neighbor.updateTotalCost();

//                     openSet.push(neighbor);
//                     openMap[neighbor].membership = OPEN;
//                     openMap[neighbor].cost = neighbor.gCost();
//                 }
//             }
//             no_of_iterations++;
//      }
        printf("NO PATH FOUND\n");
            return std::make_pair(std::vector<State>(), Seed());
    }
}
