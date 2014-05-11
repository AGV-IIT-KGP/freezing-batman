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
static const int PERMISSIBLE_INTENSITY=250;

const int DT = 1;
const int DEBUG = 1;

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

    struct comparatorMapState   {


        inline bool operator()(const StateOfCar& a, const StateOfCar& b)   {
                double k11 = a.x();
                double k12 = a.y();
                double k13 = a.theta();
                
                double cantor11 = 0.5 * (k11 + k12) * (k11 + k12 + 1) + k12;
                double cantor12 = 0.5 * (cantor11 + k13) * (cantor11 + k13 + 1) + k13;
                
                double k21 = b.x();
                double k22 = b.y();
                double k23 = b.theta();
                
                double cantor21 = 0.5 * (k21 + k22) * (k21 + k22 + 1) + k22;
                double cantor22 = 0.5 * (cantor21 + k23) * (cantor21 + k23 + 1) + k23;
                
                return cantor12 < cantor22;

        }
    };

    
    class AStarSeed
    {

    public:
        AStarSeed(const std::string& seed_file);
        std::pair<std::vector<StateOfCar>, Seed> findPathToTargetWithAstar(const cv::Mat& fusionMap, const State& start, const State& goal);
        void showPath(std::vector<StateOfCar>& path, const State& start, const State& goal);

    private:
        cv::Mat fusionMap;
        cv::Mat image;
        static const int MAX_ITERATIONS;
        const std::string SEEDS_FILE;
        std::vector<Seed> givenSeeds;
        void distanceTransform();
        bool isOnTheObstacle(const State& state);
        void loadGivenSeeds();
        void plotPointInMap(const State& pos_) ;
        std::vector<StateOfCar> neighborNodesWithSeeds(StateOfCar const& currentStateOfCar_)  ;
        bool onTarget(StateOfCar const& currentStateOfCar_, const StateOfCar& targetState)  ;
        bool isWalkableWithSeeds(StateOfCar const& startState_, StateOfCar const& nextState_) ;
        std::pair<std::vector<StateOfCar>, Seed> reconstructPath(StateOfCar const& currentStateOfCar_,  std::map<StateOfCar,StateOfCar, comparatorMapState>& came_from) ;
        void plotGrid(const State& pos_);

        

    };
}


#endif /* defined(__AStarSeed__AStarSeed__) */
