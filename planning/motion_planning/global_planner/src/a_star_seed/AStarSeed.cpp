 //
//  OldPlanner.cpp
//  OpenCV
//
//  Created by Satya Prakash on 06/09/13.
//  Copyright (c) 2013 Satya Prakash. All rights reserved.
//


#include "AStarSeed.hpp"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

namespace navigation {
    
    void addObstacles(cv::Mat& fusionMap, const int noOfObstaclesP) {
        
        fusionMap = cv::Mat::zeros(MAP_MAX, MAP_MAX,CV_8UC1);
        
        int noOfObstacles = noOfObstaclesP;
        while (noOfObstacles--) {
            int x = rand()%1000, y = rand()%1000, radius = rand()%80 + 20 ;
            cv::circle(fusionMap, cv::Point(y, x), radius, cv::Scalar(255), -1);
        }
        
    }
    
    std::pair<std::vector<StateOfCar>, Seed> AStarSeed::findPathToTargetWithAstar(const cv::Mat& img,const State&  start,const State&  goal) {
        
        fusionMap = img;
        
        image  = cv::Mat::zeros(MAP_MAX, MAP_MAX,CV_8UC1);
        StateOfCar startState(start), targetState(goal);
        std::vector<StateOfCar>emptyState(0);
        // printf("%The start point is %d %d \n", startState.x(), startState.y());
        // printf("%The start point is %d %d \n", targetState.x(), targetState.y());
        // printf("Enter to go ahead\n");
        // getchar();
        cv::circle(image, cv::Point(startState.x(), startState.y()), 5, cv::Scalar(255), -1);
        cv::circle(image, cv::Point(targetState.x(), targetState.y()), 5, cv::Scalar(255), -1);

        
        std::map<StateOfCar, open_map_element> openMap;
        std::map<StateOfCar,StateOfCar> came_from;
        
        SS::PriorityQueue<StateOfCar> openSet;
    
        openSet.push(startState);
        if (startState.isCloseTo(targetState)) {
            //            ROS_INFO("[PLANNER] Target Reached");
            std::cout<<"Bot is On Target"<<std::endl;
            return std::make_pair(emptyState, Seed());
        }
        
        while (!openSet.empty()) {

            StateOfCar currentState=openSet.top();
            if (openMap.find(currentState)!=openMap.end() && openMap[currentState].membership== CLOSED) {
                openSet.pop();
            }
            
            if (onTarget(currentState, targetState)) {
//                std::cout<<"openSet size : "<<openSet.size()<<"\n";
//                std::cout<<"Target Reached"<<std::endl;
                return reconstructPath(currentState, came_from);
            }
            openSet.pop();
//            std::cout<<"current x : "<<currentState.x()<<" current y : "<<currentState.y()<<std::endl;
//
//            plotPointInMap(currentState);
//            cv::imshow("[PLANNER] Map", mapWithObstacles);
//            cvWaitKey(0);
            openMap[currentState].membership = UNASSIGNED;
            openMap[currentState].cost=-currentState.gCost(); 
            openMap[currentState].membership=CLOSED;
            
            std::vector<StateOfCar> neighborsOfCurrentState = neighborNodesWithSeeds(currentState);
            
            
            for (unsigned int iterator=0; iterator < neighborsOfCurrentState.size(); iterator++) {
                StateOfCar neighbor = neighborsOfCurrentState[iterator];
                
                if (neighbor.isOutsideOfMap() || !isWalkableWithSeeds(currentState, neighbor)) {
                    continue;
                }
                
                double tentativeGCostAlongFollowedPath = neighbor.gCost() + currentState.gCost();
                double admissible = neighbor.distanceTo(targetState);
                double consistent = admissible;
                
                if (!((openMap.find(neighbor) != openMap.end()) &&
                      (openMap[neighbor].membership == OPEN))) {
                    came_from[neighbor] = currentState;
                    neighbor.gCost( tentativeGCostAlongFollowedPath) ;
                    neighbor.hCost( consistent) ;
                    neighbor.updateTotalCost();
                    
//                    std::cout<<"neighbor x : "<<neighbor.x()<<" neighbor y : "<<neighbor.y()<<" cost : "<<neighbor.totalCost()<<std::endl;

                    openSet.push(neighbor);
                    openMap[neighbor].membership = OPEN;
                    openMap[neighbor].cost = neighbor.gCost();
                    
                }
            }
            
        }
        std::cerr<<"NO PATH FOUND"<<std::endl;
         return std::make_pair(emptyState, Seed());
    }
    
    bool AStarSeed::onTarget(StateOfCar const& currentStateOfCar, const StateOfCar& targetState) {
        
        for (unsigned int i = 0; i < givenSeeds.size(); i++) {
            for (unsigned int j = 0; j < givenSeeds[i].intermediatePoints.size(); j++) {
                double sx = givenSeeds[i].intermediatePoints[j].x();
                double sy = givenSeeds[i].finalState.x();
                //double sz = givenSeeds[i].destination.getXcordinate();
                
                int x ((int) (currentStateOfCar.x() + sx * sin(currentStateOfCar.theta() * (CV_PI / 180)) + sy * cos(currentStateOfCar.theta() * (CV_PI / 180))));
                int y ((int) (currentStateOfCar.y() -sx * cos(currentStateOfCar.theta() * (CV_PI / 180)) + sy * sin(currentStateOfCar.theta() * (CV_PI / 180))));
                
                StateOfCar temp(State(x,y,0,0));

                if (temp.isCloseTo(targetState)) {
                    return true;
                }
            }
        }
        return false;
    }
    
    AStarSeed::AStarSeed()
    {
        loadGivenSeeds();
        
    }
    
    void AStarSeed::loadGivenSeeds() {
        const int VMAX = 70;
        const int MAX_ITER = 10000;
        const int MIN_RAD = 70;
        int numberOfSeeds;
        int return_status;
        double x, y, z;
        
        //work TODO change to c++
        FILE *textFileOFSeeds = fopen(SEEDS_FILE, "r");
        
        if (!textFileOFSeeds) {
            std::cout<<"load in opening seed file"<<std::endl;
        }
        return_status = fscanf(textFileOFSeeds, "%d\n", &numberOfSeeds);
        if (return_status == 0) {
            //ROS_ERROR("[PLANNER] Incorrect seed file format");
            //Planner::finBot();
            exit(1);
        }
        
        for (int i = 0; i < numberOfSeeds; i++) {
            Seed s;
            return_status = fscanf(textFileOFSeeds, "%lf %lf %lf %lf %lf\n", &s.velocityRatio, &x, &y, &z, &s.costOfseed);
            if (return_status == 0) {
                //                ROS_ERROR("[PLANNER] Incorrect seed file format");
                //                Planner::finBot();
                exit(1);
            }
            
            s.leftVelocity = VMAX * s.velocityRatio / (1 + s.velocityRatio);
            s.rightVelocity = VMAX / (1 + s.velocityRatio);
            //s.cost *= 1.2;
            
            s.finalState = State((int)x,(int)y,z,0);

            int n_seed_points;
            return_status = fscanf(textFileOFSeeds, "%d\n", &n_seed_points);
            if (return_status == 0) {
                std::cout<<"[PLANNER] Incorrect seed file format";
                exit(1);
            }
            
            for (int j = 0; j < n_seed_points; j++) {
                double tempXvalue,tempYvalue;
                return_status = fscanf(textFileOFSeeds, "%lf %lf \n", &tempXvalue, &tempYvalue);
                State point((int)tempXvalue, (int)tempYvalue,0 ,0);

                if (return_status == 0) {
                    //ROS_ERROR("[PLANNER] Incorrect seed file format");
                    exit(1);
                }
                
                s.intermediatePoints.insert(s.intermediatePoints.begin(), point);
            }
            givenSeeds.insert(givenSeeds.begin(), s);
        }
    }
    
    void AStarSeed::plotPointInMap(const State & pos)
    {
        cv::circle(image, cv::Point(pos.x(), pos.y()), 3, cv::Scalar(255), -1);
        
    }
    

    std::vector<StateOfCar> AStarSeed::neighborNodesWithSeeds(StateOfCar const& currentState) {
        std::vector<StateOfCar> neighbours;
        
        for ( int i = 0; i < givenSeeds.size(); i++) {
            double deltaX = givenSeeds[i].finalState.x();
            double deltaY = givenSeeds[i].finalState.y();
            double deltaZ = givenSeeds[i].finalState.theta();
            
            int x = (int) (currentState.x() +deltaX * sin(currentState.theta() * (CV_PI / 180)) + deltaY * cos(currentState.theta() * (CV_PI / 180)));
            int y=  (int) (currentState.y() -deltaX * cos(currentState.theta() * (CV_PI / 180)) + deltaY * sin(currentState.theta() * (CV_PI / 180)));
            
            double theta = (int) (deltaZ - (90 - currentState.theta())) ;
            
            StateOfCar neighbour(x,y,theta,0,givenSeeds[i].costOfseed,0,i);
            
            neighbours.push_back(neighbour);
        }
        
        
        return neighbours;
    }
        
 /*TO DO :
 isWalkable ()
 output, input should be correct */
    
    bool AStarSeed::isWalkableWithSeeds(StateOfCar const& startState, StateOfCar const& targetState) {
        
        bool NoObstacle = true;

        
        for (std::vector<State>::iterator stateIt = givenSeeds[targetState.seedTaken()].intermediatePoints.begin(); stateIt != givenSeeds[targetState.seedTaken()].intermediatePoints.end(); ++stateIt) {
            
            const State state= *stateIt;
            
            double alpha = startState.theta();
            
            int x = state.x();
            int y = state.y();
            
            int intermediateXcordinate = (int) (x * sin(alpha * (CV_PI / 180)) + y * cos(alpha * (CV_PI / 180)) + startState.x());
            
            int intermediateYcordinate = (int) (-x * cos(alpha * (CV_PI / 180)) + y * sin(alpha * (CV_PI / 180)) + startState.y());
            
            if (((0 <= intermediateXcordinate) && (intermediateXcordinate < MAP_MAX)) && ((0 <= intermediateYcordinate) && (intermediateYcordinate < MAP_MAX))) {
                
                fusionMap.at<uchar>(intermediateXcordinate,intermediateYcordinate)== 0 ? NoObstacle *= 1 : NoObstacle *= 0;
                
            } else {
                return false;
            }
        }
        
        return NoObstacle == true;
    }
    


    std::pair<std::vector<StateOfCar>, Seed> AStarSeed::reconstructPath(StateOfCar const& currentStateOfCar_, std::map<StateOfCar,StateOfCar>& came_from)
    {
        
        StateOfCar currentStateOfCar = currentStateOfCar_;
        
        std::vector<StateOfCar> path(0);
        
        path.push_back(currentStateOfCar);
        
        while (came_from.find(currentStateOfCar) != came_from.end()) {
            
            currentStateOfCar = came_from[currentStateOfCar];
            
            path.push_back(currentStateOfCar);
            
        }
        
        std::cout<<"Path size in reconstructPath is : "<<path.size()<<std::endl;
        return std::make_pair(path, givenSeeds[path[path.size()-1].seedTaken()])  ;

    }
    
    void AStarSeed::showPath(std::vector<StateOfCar>& path){
        
        
        printf("Showing A Path\n");
        for(int i = 0; i< fusionMap.rows; i++){
            for(int j = 0; j< fusionMap.cols; j++){
                image.at<uchar>(j,i)= fusionMap.at<uchar>(i,j);
            }
        }

        printf("Path size in showPath is ; %d \n", (int)path.size());
        for (std::vector<StateOfCar>::iterator stateIt = path.begin(); stateIt != path.end() ; ++stateIt) {
            const State state = *stateIt;
            plotPointInMap(state);
        }
        cv::namedWindow("obstacles Map", CV_WINDOW_NORMAL);
        cv::imshow("obstacles Map", fusionMap);
        cv::namedWindow("[PLANNER] Map", CV_WINDOW_NORMAL);
        cv::imshow("[PLANNER] Map", image);
        cvWaitKey(0);
        
    }
}