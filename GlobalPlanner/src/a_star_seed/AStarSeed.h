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

#define SEEDS_FILE "seeds2.txt"
#define OPEN 1
#define CLOSED 2
#define UNASSIGNED 3
#define VMAX 70
#define MAX_ITER 10000
#define MIN_RAD 70

namespace navigation {
    static const int GRID_SIZE =10;

//    class Triplet
//    {
//        int xValue,yValue,zValue;
//    public:
//        int getXcordinate() const
//        {
//            return xValue;
//        }
//        int getYcordinate() const
//        {
//            return yValue;
//        }
//        int getZcordinate() const
//        {
//            return zValue;
//        }
//        
//        void setXcordinate(int xVal_)
//        {
//            xValue=xVal_;
//        }
//        
//        void setYcordinate(int yVal_)
//        {
//            yValue=yVal_;
//        }
//        
//        void setZcordinate(int zVal_)
//        {
//            zValue=zVal_;
//        }
//        
//        double distanceTo(Triplet& _point)
//        {
//            return sqrt((xValue - _point.xValue) * (xValue - _point.xValue) + (yValue - _point.yValue) * (yValue - _point.yValue));
//        }
//        bool isOutsideOfMap()
//        {
//            if (!(((xValue >= 0) && (xValue < MAP_MAX)) &&
//                  ((yValue >= 0) && (yValue < MAP_MAX))))
//                return true;
//            else
//                return false;
//        }
//        
//        
//    };
    
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
    
    
//    class comparatorForPosition : public std::binary_function<Triplet, Triplet, bool> {
//    public:
//        bool operator()(Triplet const& triplet_1, Triplet const& triplet_2) const ;
//    };
    
    
    
//    class comparatorForStateOfCar
//    {
//    public:
//        bool operator() (StateOfCar const& p1,StateOfCar const& p2) const
//        {
//            if((p1.gCostAlongFollowedPath+p1.heuristicCostToGoal) > ( p2.gCostAlongFollowedPath+p2.heuristicCostToGoal) )
//                return true;
//            else
//                return false;
//        }
//    };
//    
//    class comparatorDTForStateOfCar
//    {
//    public:
//        bool operator() (StateOfCar const& p1,StateOfCar const& p2) const
//        {
//            if(p1.approxCostToGoalFromStartPointWithDistanceTransform > p2.approxCostToGoalFromStartPointWithDistanceTransform)
//                return true;
//            else
//                return false;
//        }
//    };

    
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
        cv::Mat localMap;
        cv::Mat mapWithObstacles;

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
        void addObstacles(int xValue_,int yValue_,int radius_);
        std::vector<StateOfCar> findPathToTargetWithAstar(const State& start, const State& goal);
        //        void findPathToTargetWithAstarSS();
    };
}


#endif /* defined(__OpenCV__OldPlanner__) */
