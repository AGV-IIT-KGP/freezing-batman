//
//  main.cpp
//  LocalPlanner
//
//  Created by Satya Prakash on 14/04/14.
//  Copyright (c) 2013 Satya Prakash. All rights reserved.
//


#include "local_planner.hpp"

int main(int argc, char* argv[]) {

    const std::string node_name = "local_planner";
    int confidence ;

    ros::init(argc, argv, node_name.c_str());

    ros::NodeHandle nh;
    nh.getParam("local_planner/confidence", confidence);
    navigation::LocalPlanner local_planner_seed(nh);

    if (confidence == 0) {
        local_planner_seed.plan();
    } else {
        local_planner_seed.planWithQuickReflex();
    }


    // cvNamedWindow("[PLANNER] Map", 0);

    // navigation::State botLocation(500,100,90,0),targetLocation(900,900,90,0);
    // navigation::AStarSeed planner;

    // ros::Rate loop_rate(LOOP_RATE);

    // srand((unsigned int)time(NULL));
    // struct timeval t,c;
    // int iterations = 100;
    // gettimeofday(&t,NULL);


    //     gettimeofday(&c,NULL);
    //     double td = t.tv_sec + t.tv_usec/1000000.0;
    //     double cd = c.tv_sec + c.tv_usec/1000000.0; // time in seconds for thousand iterations


    //     std::cout<<"FPS:"<< 100/(cd-td) <<std::endl;


    // ROS_INFO("Planner Exited");

    return 0;
}
