//
//  local_planner.cpp
//  LocalPlanner
//
//  Created by Satya Prakash on 28/01/14.
//  Copyright (c) 2014 Satya Prakash. All rights reserved.
//

#include <sys/time.h>
#include <local_planner.hpp>

namespace navigation {

    LocalPlanner::LocalPlanner(ros::NodeHandle& nodeHandle) : node_handle(nodeHandle) {
        node_handle.getParam("local_planner/map_max_rows", map_max_rows);
        node_handle.getParam("local_planner/map_max_cols", map_max_cols);

        fusion_map_subscriber = node_handle.subscribe("data_fuser/map", 10, &LocalPlanner::updateFusionMap, this);
        target_subscriber = node_handle.subscribe("strategy_planner/target", 10, &LocalPlanner::updateTargetPose, this);
        planning_strategy_subscriber = node_handle.subscribe("strategy_planner/which_planner", 10, &LocalPlanner::updateStrategy, this);

        seed_publisher = node_handle.advertise<local_planner::Seed>("local_planner/seed", 1000);
        status_publisher = node_handle.advertise<std_msgs::String>("local_planner/status", 1000);
        path_publisher = node_handle.advertise<nav_msgs::Path>("local_planner/path", 10);

        local_map = cv::Mat::zeros(map_max_rows, map_max_cols, CV_8UC1);
        bot_pose = navigation::State(map_max_cols / 2, map_max_rows / 10, 90, 0);
    }

    void LocalPlanner::updateFusionMap(const sensor_msgs::ImageConstPtr& world_map) {
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

    void LocalPlanner::updateStrategy(const std_msgs::String planner_strategy) {
        if (planner_strategy.data ==  std::string("A_Star_Seed")) {
           planning_strategy_=0;
        }else {
            if(planner_strategy.data==std::string("Quick_Response")){
                planning_strategy_=1;
            }            
        }
    }

    void LocalPlanner::planWithAstarSeed(navigation::AStarSeed& astar_seed_planner) {
        // struct timeval t, c;
        // gettimeofday(&t, NULL);
        std::pair<std::vector<navigation::StateOfCar>, navigation::Seed> path =
                astar_seed_planner.findPathToTarget(local_map, bot_pose, target_pose, astar_seed_planner.distance_transform, astar_seed_planner.debug_current_state, astar_seed_planner.status);
        // gettimeofday(&c, NULL);
        // double td = t.tv_sec + t.tv_usec / 1000000.0;
        // double cd = c.tv_sec + c.tv_usec / 1000000.0;

        publishData(path);
        publishStatusAStarSeed(astar_seed_planner.status);
        // publishImage(image);
    }

    void LocalPlanner::planWithQuickReflex(navigation::quickReflex& quick_reflex_planner) {
        // struct timeval t, c;
        // gettimeofday(&t, NULL);
        std::pair<std::vector<navigation::State>, navigation::Seed> path =
                quick_reflex_planner.findPathToTarget(local_map, bot_pose, target_pose, quick_reflex_planner.status);
        // gettimeofday(&c, NULL);
        // double td = t.tv_sec + t.tv_usec / 1000000.0;
        // double cd = c.tv_sec + c.tv_usec / 1000000.0;
        // std::cout<<"FPS:"<< 1/(cd-td) <<std::endl;
        // planner_quickReflex.showPath(path.first , my_bot_location, my_target_location);
        publishData(path);
        publishStatusQuickReflex(quick_reflex_planner.status);
        // publishImage(image);
    }

    void LocalPlanner::publishData(std::pair<std::vector<navigation::StateOfCar>, navigation::Seed>& path) {
        local_planner::Seed seed;
        seed.x = path.second.final_state.x();
        seed.y = path.second.final_state.y();
        seed.theta = path.second.final_state.theta();
        seed.costOfseed = path.second.costOfseed;
        seed.velocityRatio = path.second.velocityRatio;
        seed.leftVelocity = path.second.leftVelocity;
        seed.rightVelocity = path.second.rightVelocity;
        seed.curvature = path.second.final_state.curvature();
        seed_publisher.publish(seed);

        nav_msgs::Path path_msg;
        path_msg.poses.resize(path.first.size());
        for (unsigned int i = 0; i < path_msg.poses.size(); i++) {
            path_msg.poses[i].pose.position.x = path.first[i].x();
            path_msg.poses[i].pose.position.y = path.first[i].y();
        }
        path_publisher.publish(path_msg);
    }

    void LocalPlanner::publishData(std::pair<std::vector<navigation::State>, navigation::Seed>& path) {
        local_planner::Seed seed;
        seed.x = path.second.final_state.x();
        seed.y = path.second.final_state.y();
        seed.theta = path.second.final_state.theta();
        seed.costOfseed = path.second.costOfseed;
        seed.velocityRatio = path.second.velocityRatio;
        seed.leftVelocity = path.second.leftVelocity;
        seed.rightVelocity = path.second.rightVelocity;
        seed.curvature = path.second.final_state.curvature();
        seed_publisher.publish(seed);

        nav_msgs::Path path_msg;
        path_msg.poses.resize(path.first.size());
        for (unsigned int i = 0; i < path_msg.poses.size(); i++) {
            path_msg.poses[i].pose.position.x = bot_pose.x() + path.first[i].x();
            path_msg.poses[i].pose.position.y = bot_pose.y() + path.first[i].y();
        }
        path_publisher.publish(path_msg);
    }

    void LocalPlanner::publishImage(cv::Mat image) {
        cv_bridge::CvImage out_msg;
        out_msg.encoding = sensor_msgs::image_encodings::TYPE_8UC1;
        out_msg.image = image;
        pub_path_image.publish(out_msg.toImageMsg());
    }

    void LocalPlanner::publishStatusQuickReflex(int status) {
        std_msgs::String msg;
        std::stringstream ss;

        if (status == 0) {
            ss << "NO PATH FOUND";
        } else if (status == 1) {
            ss << "BOT ON TARGET";
        } else {
            ss << "A PATH IS FOUND";
        }

        msg.data = ss.str();
        status_publisher.publish(msg);
    }

    void LocalPlanner::publishStatusAStarSeed(int status) {
        std_msgs::String msg;
        std::stringstream ss;

        if (status == 0) {
            ss << "NO PATH FOUND";
        } else if (status == 1) {
            ss << "BOT ON TARGET";
        } else if (status == 2) {
            ss << "PATH FOUND";
        } else {
            ss << "OPEN LIST OVERFLOW";
        }

        msg.data = ss.str();
        status_publisher.publish(msg);
    }

    
}