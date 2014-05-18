//
//  local_planner.cpp
//  LocalPlanner
//
//  Created by Satya Prakash on 28/01/14.
//  Copyright (c) 2014 Satya Prakash. All rights reserved.
//

#include "local_planner.hpp"
#include <sys/time.h>

namespace navigation {

    LocalPlanner::LocalPlanner(ros::NodeHandle& nodeHandle) : nh(nodeHandle) {
    loadParams(nh);
        int MAP_MAX_COLS, MAP_MAX_ROWS;
        nh.getParam("local_planner/map_max_rows", MAP_MAX_ROWS);
        nh.getParam("local_planner/map_max_cols", MAP_MAX_COLS);

        sub_world_map = nh.subscribe("data_fuser/map", 10, &LocalPlanner::updateWorldMap, this);
        sub_target_pose = nh.subscribe("strategy_planner/target", 10, &LocalPlanner::updateTargetPose, this);

        pub_seed = nh.advertise<local_planner::Seed>("local_planner/seed", 1000);
        pub_status = nh.advertise<std_msgs::String>("local_planner/status", 1000);
        pub_nav_msgs = nh.advertise<nav_msgs::Path>("local_planner/path", 10);

        local_map = cv::Mat::zeros(MAP_MAX_ROWS, MAP_MAX_COLS, CV_8UC1);
        my_bot_location = navigation::State(MAP_MAX_COLS/2, MAP_MAX_ROWS/10, 90, 0);

    }

    void LocalPlanner::updateWorldMap(const sensor_msgs::ImageConstPtr& world_map) {
        //TODO : copy function for occupancy grid
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(world_map, sensor_msgs::image_encodings::MONO8);
            local_map = cv_ptr->image;
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
    }

    void LocalPlanner::planWithAstarSeed() {
        
        int LOOP_RATE;
        nh.getParam("local_planner/loop_rate", LOOP_RATE);
        ros::Rate loop_rate(LOOP_RATE);
        navigation::AStarSeed planner_AStarSeed(nh);

        while (ros::ok()) {
            ros::spinOnce();

            // struct timeval t, c;
            // gettimeofday(&t, NULL);
            std::pair<std::vector<navigation::StateOfCar>, navigation::Seed> path =
                    planner_AStarSeed.findPathToTarget(local_map, my_bot_location, my_target_location, planner_AStarSeed.distance_transform, planner_AStarSeed.debug_current_state, planner_AStarSeed.status);
            // gettimeofday(&c, NULL);
            // double td = t.tv_sec + t.tv_usec / 1000000.0;
            // double cd = c.tv_sec + c.tv_usec / 1000000.0;

            publishData(path);
            publishStatusAStarSeed(planner_AStarSeed.status);
            // publishImage(image);
            loop_rate.sleep();
        }
    }

    void LocalPlanner::planWithQuickReflex() {
        int LOOP_RATE;
        nh.getParam("local_planner/loop_rate", LOOP_RATE);
        ros::Rate loop_rate(LOOP_RATE);
        navigation::quickReflex planner_quickReflex(nh);

        while (ros::ok()) {
            ros::spinOnce();

            // struct timeval t, c;
            // gettimeofday(&t, NULL);
            std::pair<std::vector<navigation::State>, navigation::Seed> path =
                    planner_quickReflex.findPathToTarget(local_map, my_bot_location, my_target_location, planner_quickReflex.status);
            // gettimeofday(&c, NULL);
            // double td = t.tv_sec + t.tv_usec / 1000000.0;
            // double cd = c.tv_sec + c.tv_usec / 1000000.0;
            // std::cout<<"FPS:"<< 1/(cd-td) <<std::endl;
            // planner_quickReflex.showPath(path.first , my_bot_location, my_target_location);
            publishData(path);
            publishStatusQuickReflex(planner_quickReflex.status);
            // publishImage(image);
            loop_rate.sleep();
        }
    }

    void LocalPlanner::publishData(std::pair<std::vector<navigation::StateOfCar>, navigation::Seed>& path) {
        local_planner::Seed seed;

        seed.x = path.second.finalState.x();
        seed.y = path.second.finalState.y();
        seed.theta = path.second.finalState.theta();
        seed.costOfseed = path.second.costOfseed;
        seed.velocityRatio = path.second.velocityRatio;
        seed.leftVelocity = path.second.leftVelocity;
        seed.rightVelocity = path.second.rightVelocity;
        seed.curvature = path.second.finalState.curvature();

        pub_seed.publish(seed);

        nav_msgs::Path path_msg;
        path_msg.poses.resize(path.first.size());
        for (unsigned int i = 0; i < path_msg.poses.size(); i++) {
            path_msg.poses[i].pose.position.x = path.first[i].x();
            path_msg.poses[i].pose.position.y = path.first[i].y();
        }
        pub_nav_msgs.publish(path_msg);
    }

    void LocalPlanner::publishData(std::pair<std::vector<navigation::State>, navigation::Seed>& path) {
        local_planner::Seed seed;

        seed.x = path.second.finalState.x();
        seed.y = path.second.finalState.y();
        seed.theta = path.second.finalState.theta();
        seed.costOfseed = path.second.costOfseed;
        seed.velocityRatio = path.second.velocityRatio;
        seed.leftVelocity = path.second.leftVelocity;
        seed.rightVelocity = path.second.rightVelocity;
        seed.curvature = path.second.finalState.curvature();

        pub_seed.publish(seed);

        nav_msgs::Path path_msg;
        path_msg.poses.resize(path.first.size());
        for (unsigned int i = 0; i < path_msg.poses.size(); i++) {
            path_msg.poses[i].pose.position.x = my_bot_location.x() + path.first[i].x();
            path_msg.poses[i].pose.position.y = my_bot_location.y() + path.first[i].y();
        }
        pub_nav_msgs.publish(path_msg);
    }

    void LocalPlanner::publishImage(cv::Mat image) {
        cv_bridge::CvImage out_msg;
        out_msg.encoding = sensor_msgs::image_encodings::TYPE_8UC1;
        out_msg.image = image;
        pub_path_image.publish(out_msg.toImageMsg());

    }

    void LocalPlanner::publishStatusQuickReflex(int status){
        std_msgs::String msg;

        std::stringstream ss;

        if(status == 0){
            ss << "NO PATH FOUND";
        }
        else if(status == 1){
            ss << "BOT ON TARGET";
        }
        else {
            ss << "A PATH IS FOUND";
        }

        msg.data = ss.str();
        pub_status.publish(msg);
    }

    void LocalPlanner::publishStatusAStarSeed(int status) {
        
        std_msgs::String msg;

        std::stringstream ss;
        if (status == 0) {
            ss << "NO PATH FOUND";
        } 
        else if (status == 1) {
            ss << "BOT ON TARGET";
        }
        else if (status == 2) {
            ss << "PATH FOUND";
        }
        else {
            ss << "OVERFLOW : OPEN LIST SIZE " << status;
        }

        msg.data = ss.str();

        pub_status.publish(msg);
    }

    void LocalPlanner::loadParams(ros::NodeHandle& nh) {
    planning_strategy = 0;
    std::string node_name = std::string("/") + ros::this_node::getName();
    nh.getParam(node_name + "/planning_strategy", planning_strategy);
    
}

}