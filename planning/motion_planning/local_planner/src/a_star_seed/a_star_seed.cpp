//
//  a_star_seed.cpp
//  AStarSeed
//
//  Created by Satya Prakash on 06/09/13.
//  Copyright (c) 2013 Satya Prakash. All rights reserved.
//


#include "a_star_seed/a_star_seed.hpp"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

namespace navigation {
    


    const int AStarSeed::MAX_ITERATIONS = 1000;

    void addObstacles(cv::Mat& img, const int noOfObstaclesP = 0) {
        

        srand((unsigned int)time(NULL));
        cv::circle(img, cv::Point(500, img.rows - 300 -1), 100, cv::Scalar(255), -1);

        int noOfObstacles = noOfObstaclesP;
        while (noOfObstacles--) {

            const int x      = rand()%800;
            const int y      = rand()%800;
            const int radius = rand()%40 + 20 ;

            const int x1      = rand()%100;
            const int y1      = rand()%100;
            cv::circle(img, cv::Point(x, img.rows - y -1), radius, cv::Scalar(255), -1);

        }
        
    }
    

    bool AStarSeed::isOnTheObstacle(const State& state){
        return fusionMap.at<uchar>(fusionMap.rows - state.y() -1, state.x()) != 0;
    }

    std::pair<std::vector<StateOfCar>, Seed> AStarSeed::findPathToTargetWithAstar(const cv::Mat& img,const State&  start,const State&  goal) {
        

        // USE : for garanteed termination of planner
        int no_of_iterations = 0;

        fusionMap = img;
        
        image = fusionMap - fusionMap;

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

        while (!openSet.empty()) {
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

            // TODO : use closeTo instead of onTarget
            if (onTarget(currentState, targetState)) {
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
                
                if (!((openMap.find(neighbor) != openMap.end()) &&
                      (openMap[neighbor].membership == OPEN))) {
                    came_from[neighbor] = currentState;
                    neighbor.gCost( tentativeGCostAlongFollowedPath) ;
                    neighbor.hCost( consistent) ;
                    neighbor.updateTotalCost();
                    

                    openSet.push(neighbor);
                    openMap[neighbor].membership = OPEN;
                    openMap[neighbor].cost = neighbor.gCost();
                    
                }
            }
            no_of_iterations++;
        }
        // std::cerr<<"NO PATH FOUND"<<std::endl;
            return std::make_pair(std::vector<StateOfCar>(), Seed());
    }
    
    bool AStarSeed::onTarget(StateOfCar const& currentState, const StateOfCar& targetState) {
        
        for (unsigned int i = 0; i < givenSeeds.size(); i++) {
            for (unsigned int j = 0; j < givenSeeds[i].intermediatePoints.size(); j++) {
                const double sx = givenSeeds[i].intermediatePoints[j].x();
                const double sy = givenSeeds[i].finalState.x();
                //double sz = givenSeeds[i].destination.getXcordinate();
                
                const int x ((int) (currentState.x() + sx * sin(currentState.theta() * (CV_PI / 180)) + sy * cos(currentState.theta() * (CV_PI / 180))));
                const int y ((int) (currentState.y() -sx * cos(currentState.theta() * (CV_PI / 180)) + sy * sin(currentState.theta() * (CV_PI / 180))));
                
                StateOfCar temp(State(x,y,0,0));

                if (temp.isCloseTo(targetState)) {
                    return true;
                }
            }
        }
        return false;
    }
    
    AStarSeed::AStarSeed(const std::string& seed_file) : SEEDS_FILE("/home/agv/fuerte_workspace/sandbox/freezing-batman/planning/motion_planning/local_planner/seeds/seeds2.txt")
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
        FILE *textFileOFSeeds = fopen(SEEDS_FILE.c_str(), "r");
        
        if (!textFileOFSeeds) {
            std::cout<<"load in opening seed file : "<<SEEDS_FILE<<std::endl;
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
            givenSeeds.push_back(s);
        }
    }
    
    void AStarSeed::plotPointInMap(const State & pos)
    {
        cv::circle(image, cv::Point(pos.x(), image.rows- pos.y() - 1), 3, cv::Scalar(255), -1);
        
    }
    

    std::vector<StateOfCar> AStarSeed::neighborNodesWithSeeds(StateOfCar const& currentState) {
        std::vector<StateOfCar> neighbours;
        
        for ( int i = 0; i < givenSeeds.size(); i++) {
            const double deltaX = givenSeeds[i].finalState.x();
            const double deltaY = givenSeeds[i].finalState.y();
            const double deltaZ = givenSeeds[i].finalState.theta();
            
            const int x = (int) (currentState.x() +deltaX * sin(currentState.theta() * (CV_PI / 180)) + deltaY * cos(currentState.theta() * (CV_PI / 180)));
            const int y=  (int) (currentState.y() -deltaX * cos(currentState.theta() * (CV_PI / 180)) + deltaY * sin(currentState.theta() * (CV_PI / 180)));
            
            const double theta = (int) (deltaZ - (90 - currentState.theta())) ;
            
            const StateOfCar neighbour(x,y,theta,0,givenSeeds[i].costOfseed,0,i);
            
     
            if ( !isWalkableWithSeeds(currentState, neighbour)) {
                continue;
            }
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
            
            if (((intermediateXcordinate >= 0) && (intermediateXcordinate < MAP_MAX)) && (( intermediateYcordinate >= 0) && (intermediateYcordinate < MAP_MAX))) {

                fusionMap.at<uchar>(fusionMap.rows - intermediateYcordinate -1, intermediateXcordinate) < 128 ? NoObstacle *= 1 : NoObstacle *= 0;
    

            } else {
                return false;
            }
        }
        
        if(fusionMap.at<uchar>(fusionMap.rows - targetState.y() -1, targetState.x()) != 0)
            return false;
        return NoObstacle == true;
    }
    


    std::pair<std::vector<StateOfCar>, Seed> AStarSeed::reconstructPath(StateOfCar const& currentStateOfCar_, std::map<StateOfCar,StateOfCar, comparatorMapState>& came_from)   {
        
        StateOfCar currentStateOfCar = currentStateOfCar_;
        
        std::vector<StateOfCar> path;
        
        path.push_back(currentStateOfCar);
        
        while (came_from.find(currentStateOfCar) != came_from.end()) {
            
            currentStateOfCar = came_from[currentStateOfCar];
        
            path.push_back(currentStateOfCar);

        }
        
        if(path.size() < 2) 
            return std::make_pair(path, Seed());
        return std::make_pair(path, givenSeeds[path[path.size()-2].seedTaken()])  ;

    }
    
    void AStarSeed::showPath(std::vector<StateOfCar>& path) {
        

        
        printf("Showing A Path\n");
        for(int i = 0; i< fusionMap.rows; i++){
            for(int j = 0; j< fusionMap.cols; j++){
                image.at<uchar>(i,j)= fusionMap.at<uchar>(i,j);
            }
        }

        printf("Path size in showPath is ; %d \n", (int)path.size());
        for (std::vector<StateOfCar>::iterator stateIt = path.begin(); stateIt != path.end() ; ++stateIt) {
            const State state = *stateIt;
            plotPointInMap(state);
        }
        cv::imshow("[PLANNER] Map", image);
        cvWaitKey(0);
        
    }
}
