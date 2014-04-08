 //
//  OldPlanner.cpp
//  OpenCV
//
//  Created by Satya Prakash on 06/09/13.
//  Copyright (c) 2013 Satya Prakash. All rights reserved.
//


#include "AStarSeed.h"

namespace navigation {
    
    std::vector<StateOfCar> AStarSeed::findPathToTargetWithAstar(const State&  start,const State&  goal) {
        
        
        StateOfCar startState(start), targetState(goal);
        
        plotPointInMap(startState);
        plotPointInMap(targetState);
        
        std::map<StateOfCar, open_map_element> openMap;
        std::map<StateOfCar,StateOfCar> came_from;
        sPriorityQueue<StateOfCar> openSet;
    
        openSet.push(startState);
        if (startState.isCloseTo(targetState)) {
            //            ROS_INFO("[PLANNER] Target Reached");
            std::cout<<"Bot is On Target"<<std::endl;
            return std::vector<StateOfCar>();
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
        return std::vector<StateOfCar>();
    }
    
    bool AStarSeed::onTarget(StateOfCar const& currentStateOfCar, const StateOfCar& targetState) {
        for (unsigned int i = 0; i < givenSeeds.size(); i++) {
            for (unsigned int j = 0; j < givenSeeds[i].intermediatePoints.size(); j++) {
                double sx = givenSeeds[i].intermediatePoints[j].x();
                double sy = givenSeeds[i].finalState.x();
                //double sz = givenSeeds[i].destination.getXcordinate();
                
                int x((int) (currentStateOfCar.x() +
                                                        sx * sin(currentStateOfCar.theta() * (CV_PI / 180)) +
                                                        sy * cos(currentStateOfCar.theta() * (CV_PI / 180))));
                int y = ((int) (currentStateOfCar.y() +
                                                        -sx * cos(currentStateOfCar.theta() * (CV_PI / 180)) +
                                                        sy * sin(currentStateOfCar.theta() * (CV_PI / 180))));
                
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
        localMap = cv::Mat::zeros(MAP_MAX, MAP_MAX,CV_8UC1);
        mapWithObstacles=cv::Mat::zeros(MAP_MAX, MAP_MAX,CV_8UC3);
        
    }
    
    void AStarSeed::loadGivenSeeds() {
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
        int xValue = pos.x();
        int yValue = MAP_MAX - pos.y() - 1;
        int xValueInMap = xValue > MAP_MAX ? MAP_MAX - 1 : xValue ;
        xValueInMap = xValue < 0 ? 0 : xValue;
        int yValueInMap = yValue > MAP_MAX ? MAP_MAX - 1 : yValue;
        yValueInMap = yValue < 0 ? 0 : yValue;
        
        int xValueInMap_ = xValueInMap;
        int yValueInMap_ = yValue + 5 > MAP_MAX ? MAP_MAX - 1 : yValue + 5;
        yValueInMap_ = yValue + 5 < 0 ? 0 : yValue + 5;
        
        srand((unsigned int) time(NULL));
        cv::line(mapWithObstacles,cvPoint(xValueInMap, yValueInMap),cvPoint(xValueInMap_,yValueInMap_),CV_RGB(127, 127, 127),2,CV_AA,0);
        
    }
    

    std::vector<StateOfCar> AStarSeed::neighborNodesWithSeeds(StateOfCar const& currentStateOfCar) {
        std::vector<StateOfCar> neighbours;
        
        for ( int i = 0; i < givenSeeds.size(); i++) {
            double changeInXcordinate = givenSeeds[i].finalState.x();
            double changeInYcordinate = givenSeeds[i].finalState.y();
            double changeInZcordinate = givenSeeds[i].finalState.theta();
            
            int x = (int) (currentStateOfCar.x() +
                                                         changeInXcordinate * sin(currentStateOfCar.theta() * (CV_PI / 180)) +
                                                         changeInYcordinate * cos(currentStateOfCar.theta() * (CV_PI / 180)));
            int y=  (int) (currentStateOfCar.y() +
                                                          -changeInXcordinate * cos(currentStateOfCar.theta() * (CV_PI / 180)) +
                                                          changeInYcordinate * sin(currentStateOfCar.theta() * (CV_PI / 180)));
            
            double theta = (int) (changeInZcordinate - (90 - currentStateOfCar.theta())) ;
            
            StateOfCar neighbour(x,y,theta,0,givenSeeds[i].costOfseed,0,i);
            
            neighbours.push_back(neighbour);
        }
        
        
        return neighbours;
    }
        
 
    
    bool AStarSeed::isWalkableWithSeeds(StateOfCar const& startState, StateOfCar const& targetState) {
        
        bool ObstacleIsPresent = true;

        
        for (auto state : givenSeeds[targetState.seedTaken()].intermediatePoints) {
            int intermediateXcordinate, intermediateYcondinate;
            double alpha = startState.theta();
            
            int seedIntermediateXvalue, seedIntermediateYvalue;
            seedIntermediateXvalue = state.x();
            seedIntermediateYvalue = state.y();
            
            intermediateXcordinate = (int) (seedIntermediateXvalue * sin(alpha * (CV_PI / 180)) + seedIntermediateYvalue * cos(alpha * (CV_PI / 180)) + startState.x());
            intermediateYcondinate = (int) (-seedIntermediateXvalue * cos(alpha * (CV_PI / 180)) + seedIntermediateYvalue * sin(alpha * (CV_PI / 180)) + startState.y());
            
            if (((0 <= intermediateXcordinate) && (intermediateXcordinate < MAP_MAX)) && ((0 <= intermediateYcondinate) && (intermediateYcondinate < MAP_MAX))) {
                localMap.at<uchar>(intermediateXcordinate,intermediateYcondinate)== 0 ? ObstacleIsPresent *= 1 : ObstacleIsPresent *= 0;
            } else {
                return false;
            }
        }
        
        return ObstacleIsPresent == 1;
    }
    

    void AStarSeed::addObstacles(const int noOfObstaclesP) {
        
        localMap = cv::Mat::zeros(MAP_MAX, MAP_MAX,CV_8UC1);
        mapWithObstacles=cv::Mat::zeros(MAP_MAX, MAP_MAX,CV_8UC3);
        
        int noOfObstacles = noOfObstaclesP;
        while (noOfObstacles--) {
            int x = rand()%1000, y = rand()%1000, radius = rand()%80 + 20 ;
            cv::circle(localMap, cv::Point(y, x), radius, cv::Scalar(255), -1);
            cv::circle(mapWithObstacles, cv::Point(x, MAP_MAX - y -1), radius, cv::Scalar(255, 255, 255), -1, CV_AA, 0);
        }
        
    }
    
    

    std::vector<StateOfCar> AStarSeed::reconstructPath(StateOfCar const& currentStateOfCar_, std::map<StateOfCar,StateOfCar>& came_from)
    {
        
        StateOfCar currentStateOfCar = currentStateOfCar_;
        
        std::vector<StateOfCar> path ;
        
        path.push_back(currentStateOfCar);
        
        while (came_from.find(currentStateOfCar) != came_from.end()) {
            
            currentStateOfCar = came_from[currentStateOfCar];
            
            path.push_back(currentStateOfCar);
            
        }
        
        return path;

    }
    
    void AStarSeed::showPath(std::vector<StateOfCar>& path){
        
        for (auto state : path) {
            plotPointInMap(state);
        }
        cv::imshow("obstacles Map", localMap);

        cv::imshow("[PLANNER] Map", mapWithObstacles);
        cvWaitKey(0);
        
    }
}