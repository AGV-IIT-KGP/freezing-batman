//
//  a_star_seed.hpp
//  AStarSeed
//
//  Created by Satya Prakash on 28/01/14.
//  Copyright (c) 2014 Satya Prakash. All rights reserved.
//
#ifndef __AStarSeed__AStarSeed__
#define __AStarSeed__AStarSeed__


#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include <sstream>
#include <queue>
#include <map>
#include <string>

#include "ss_priority_queue.hpp"
#include "state_of_car.hpp"
#include "seed.hpp"

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

    
    
    class AStarSeed : public Planning::Planner
    {
        static const int MAX_ITERATIONS;
        const std::string SEEDS_FILE;
    public:
        cv::Mat fusionMap;
        cv::Mat image;

        std::vector<Seed> givenSeeds;
        bool isOnTheObstacle(const State& state);

        void loadGivenSeeds();
        void plotPointInMap(const State& pos_) ;
        std::vector<StateOfCar> neighborNodesWithSeeds(StateOfCar const& currentStateOfCar_)  ;
        bool onTarget(StateOfCar const& currentStateOfCar_, const StateOfCar& targetState)  ;
        bool isWalkableWithSeeds(StateOfCar const& startState_, StateOfCar const& nextState_) ;
        std::pair<std::vector<StateOfCar>, Seed> reconstructPath(StateOfCar const& currentStateOfCar_,  std::map<StateOfCar,StateOfCar>& came_from) ;
        void plotGrid(const State& pos_);
        void showPath(std::vector<StateOfCar>& path);

        
    public:
        AStarSeed();

        std::pair<std::vector<StateOfCar>, Seed> findPathToTargetWithAstar(const cv::Mat& fusionMap, const State& start, const State& goal);
        //        void findPathToTargetWithAstarSS();
    };
}


#endif /* defined(__AStarSeed__AStarSeed__) */
