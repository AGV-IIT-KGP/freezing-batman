//
//  a_star_seed_tester.cpp
//  AStarSeed
//
//  Created by Satya Prakash on 27/03/14.
//  Copyright (c) 2014 Satya Prakash. All rights reserved.
//


#include <iostream>
#include <sys/time.h>
#include "a_star_seed/a_star_seed.hpp"

int main(){
    
    navigation::State botLocation(rand()%100,rand()%100,90,0),targetLocation(900,900,90,0);
    
    srand((unsigned int)time(NULL));

    struct timeval t,c;
    gettimeofday(&t,NULL);
    
    int iterations = 1000;

    navigation::AStarSeed planner;

    
    while (iterations--) {

        
        cv::Mat img = cv::Mat::zeros(1000, 1000, CV_8UC1);
        // std::chrono::steady_clock::time_point startC=std::chrono::steady_clock::now();

        navigation::addObstacles(img, 5);
        
        std::vector<navigation::StateOfCar> path = planner.findPathToTargetWithAstar(img,botLocation, targetLocation);
        
        // std::chrono::steady_clock::time_point endC=std::chrono::steady_clock::now();

        planner.showPath(path);
        // long long takenTime=std::chrono::duration_cast<std::chrono::microseconds > (endC - startC).count();
        // std::cout<<"Taken time: "<<takenTime<<std::endl;

        // std::cout<<"FPS : "<<(1000000.0)/takenTime<<std::endl;

    }

    
    gettimeofday(&c,NULL);
    double td = t.tv_sec + t.tv_usec/1000000.0;
    double cd = c.tv_sec + c.tv_usec/1000000.0;
    std::cout<<"FPS:"<< 1000.0/(cd-td) <<std::endl;

}