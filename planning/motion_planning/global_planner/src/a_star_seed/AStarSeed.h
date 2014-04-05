//
//  OldPlanner.h
//  OpenCV
//
//  Created by Satya Prakash on 06/09/13.
//  Copyright (c) 2013 Satya Prakash. All rights reserved.
//

#ifndef __OpenCV__OldPlanner__
#define __OpenCV__OldPlanner__


#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include <sstream>
#include <queue>
#include <map>
#include <unordered_map>
// #include "SSPriority_queue.hpp"
#include "StateOfCar.hpp"
#define LEFT_CMD 0
#define RIGHT_CMD 1

#define SEEDS_FILE "/home/krishna/Downloads/seeds2.txt"
#define OPEN 1
#define CLOSED 2
#define UNASSIGNED 3
#define VMAX 70
#define MAX_ITER 10000
#define MIN_RAD 70

namespace navigation {
    static const int GRID_SIZE =10;

    void addObstacles(cv::Mat& img, const int noOfObstacles);
    

    
    
    class open_map_element {
    public:
        char membership;
        double cost;
    };

    
    
    
    class Seed{
    public:
        State finalState;
        double costOfseed,velocityRatio,leftVelocity,rightVelocity;
        std::vector<State> intermediatePoints;
    };
    


    
    template < typename T,typename Sequence = std::vector<T> , typename CompareFunction = std::less<typename Sequence::value_type > >
    class sPriorityQueue : public std::priority_queue<T,Sequence, CompareFunction>
    {
    public:
        void clear()
        {
            this->c.clear();
            
        }
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
        std::vector<StateOfCar> reconstructPath(StateOfCar const& currentStateOfCar_,  std::map<StateOfCar,StateOfCar>& came_from) ;
        void plotGrid(const State& pos_);
        void showPath(std::vector<StateOfCar>& path);

        
    public:
        AStarSeed();

        std::vector<StateOfCar> findPathToTargetWithAstar(const cv::Mat& fusionMap, const State& start, const State& goal);
        //        void findPathToTargetWithAstarSS();
    };
}


#endif /* defined(__OpenCV__OldPlanner__) */
