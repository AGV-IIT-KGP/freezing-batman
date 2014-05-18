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
#include <stdio.h>


#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"


#include "ss_priority_queue.hpp"
#include "state_of_car.hpp"
#include "seed.hpp"
static const int PERMISSIBLE_INTENSITY=250;



namespace navigation {

    void addObstacles(cv::Mat& img, const int noOfObstacles);

    enum MEMBERSHIP {
        OPEN = 1, CLOSED = 2, UNASSIGNED = 3
    };

    class open_map_element {
    public:
        char membership;
        double cost;
    };

    struct comparatorMapState {

        inline bool operator()(const StateOfCar& a, const StateOfCar& b) {
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

    class AStarSeed {
    public:
        AStarSeed( ros::NodeHandle& nodehandle);
        std::pair<std::vector<StateOfCar>, Seed> findPathToTarget(const cv::Mat& fusionMap, const State& start, const State& goal, int , int, int&);
        int distance_transform, debug_current_state;
        std::string getSeedFileNameAStarSeed(ros::NodeHandle& nodeHandle);
        ros::NodeHandle nh;
        int status;


    private:
        static const int MAX_ITERATIONS;
        std::string SEEDS_FILE;
        int MAP_MAX_COLS;
        int MAP_MAX_ROWS;
        std::vector<Seed> givenSeeds;
        cv::Mat fusionMap;
        cv::Mat image;

        void distanceTransform();
        void loadGivenSeeds();
        void plotPointInMap(const State& pos_);
        void plotGrid(const State& pos_);
        bool onTarget(StateOfCar const& currentStateOfCar_, const StateOfCar& targetState);
        bool isOnTheObstacle(const State& state);
        bool isWalkableWithSeeds(const StateOfCar& startState_, const StateOfCar& nextState_, int MAP_MAX_COLS, int MAP_MAX_ROWS);
        std::vector<StateOfCar> neighborNodesWithSeeds(const StateOfCar& currentStateOfCar_);
        std::pair<std::vector<StateOfCar>, Seed> reconstructPath(const StateOfCar& currentStateOfCar_, std::map<StateOfCar, StateOfCar, comparatorMapState>& came_from);
        

    };

    class quickReflex
    {

    public:
        quickReflex(ros::NodeHandle& nodehandle);
        quickReflex(const State& start, const State& goal){
            loadGivenSeeds(start, goal);
        };
        std::pair<std::vector<State>, Seed> findPathToTarget(const cv::Mat& fusionMap, const State& start, const State& goal, int&);
        void showPath(std::vector<State>& path, const State& start, const State& goal);
        std::string getSeedFileNameQuickReflex(ros::NodeHandle& nodeHandle);
        ros::NodeHandle nh;
        int status;

    private:
        cv::Mat fusionMap;
        cv::Mat image;
        std::string SEEDS_FILE;

        void distanceTransform();
        void loadGivenSeeds(const State& start, const State& goal);
        void plotPointInMap(const State& pos_) ;
        bool isOnTheObstacle(const State& state);
        bool onTarget(State const& currentState_, const State& targetState)  ;
        bool isWalkableWithSeeds(State const& startState_, State const& nextState_, Seed targetSeed) ;
        std::vector<Seed> givenSeeds;
        std::vector<Seed> neighborNodesWithSeeds(const State&  start,const State&  goal)  ;

    };


}


#endif /* defined(__AStarSeed__AStarSeed__) */
