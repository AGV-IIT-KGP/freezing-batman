//
//  AStarSeed.hpp
//  GlobalPlanner
//
//  Created by Satya Prakash on 06/09/13.
//  Copyright (c) 2013 Satya Prakash. All rights reserved.
//

#ifndef __OpenCV__AStarSeed__
#define __OpenCV__AStarSeed__


#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include <sstream>
#include <queue>
#include <map>
#include "SSPriority_queue.hpp"
#include "StateOfCar.hpp"
#include "Seed.hpp"

#define SEEDS_FILE "/home/krishna/Downloads/seeds2.txt"


namespace navigation {

    void addObstacles(cv::Mat& img, const int noOfObstacles);
    
    
    enum MEMBERSHIP {
        OPEN =1, CLOSED =2, UNASSIGNED=3
    };
    
    class open_map_element {
    public:
        char membership;
        double cost;
    };

    
    
    class AStarSeed
    {
    public:
        cv::Mat fusionMap;
        cv::Mat image;

        std::vector<Seed> givenSeeds;
        
        void loadGivenSeeds();
        void plotPointInMap(const State& pos_) ;
        std::vector<StateOfCar> neighborNodesWithSeeds(StateOfCar const& currentStateOfCar_)  ;
        bool onTarget(StateOfCar const& currentStateOfCar_, const StateOfCar& targetState)  ;
        bool isWalkableWithSeeds(StateOfCar const& startState_, StateOfCar const& nextState_) ;
        std::pair<std::vector<StateOfCar>, Seed>     reconstructPath(StateOfCar const& currentStateOfCar_,  std::map<StateOfCar,StateOfCar>& came_from) ;
        void plotGrid(const State& pos_);
        void showPath(std::vector<StateOfCar>& path);

        
    public:
        AStarSeed();

        std::pair<std::vector<StateOfCar>, Seed> findPathToTargetWithAstar(const cv::Mat& fusionMap, const State& start, const State& goal);
        //        void findPathToTargetWithAstarSS();
    };
}


#endif /* defined(__OpenCV__AStarSeed__) */
