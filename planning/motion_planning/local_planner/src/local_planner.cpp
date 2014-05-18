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
        //Subscriber for World Map
        sub_world_map = node_handle.subscribe("interpreter/fusion/world_map", 10, &LocalPlanner::updateWorldMap, this);
        // topic should same with data published by GPS
        sub_bot_pose = node_handle.subscribe("/bot_pose", 10, &LocalPlanner::updateBotPose, this);
        // topic published from GPS
        sub_target_pose = node_handle.subscribe("/target_pose", 10, &LocalPlanner::updateTargetPose, this);

        pub_path = node_handle.advertise<local_planner::Seed>("/path", 1000); //Publisher for Path
        // it = new image_transport::ImageTransport(nh);
        // pub_path_image= it->advertise("/pathImage", 1); //Publisher for final path image

        pub_nav_msgs = node_handle.advertise<nav_msgs::Path>("/path_nav_msgs", 10); //nav_msgs for path

        local_map = cv::Mat::zeros(MAP_MAX, MAP_MAX, CV_8UC1);

        ROS_INFO("Local Planner(AStarSeed) started.... ");
        ROS_INFO("Publisher : \"/path\" .... ");
        // ROS_INFO("Publisher : \"/pathImage\" .... ");
        ROS_INFO("Subscriber : \"interpreter/fusion/world_map\", \"/bot_pose\", \"/target_pose\" .... ");
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

    void LocalPlanner::plan() {
        ros::Rate loop_rate(LOOP_RATE);
        navigation::AStarSeed planner(node_handle);

        while (ros::ok()) {
            ros::spinOnce();

            struct timeval t, c;
            gettimeofday(&t, NULL);
            std::pair<std::vector<navigation::StateOfCar>, navigation::Seed> path =
                    planner.findPathToTargetWithAstar(local_map, my_bot_location, my_target_location, planner.distance_transform, planner.debug_current_state);

            // cv::Mat image = planner.showPath(path.first, my_bot_location, my_target_location);
            gettimeofday(&c, NULL);
            double td = t.tv_sec + t.tv_usec / 1000000.0;
            double cd = c.tv_sec + c.tv_usec / 1000000.0;

            publishData(path);
            // publishImage(image);
            loop_rate.sleep();
        }
    }

    void LocalPlanner::planWithQuickReflex() {
        ros::Rate loop_rate(LOOP_RATE);
        navigation::quickReflex quick_reflex_planner(node_handle);

        while (ros::ok()) {
            ros::spinOnce();

            struct timeval t, c;
            gettimeofday(&t, NULL);
            std::pair<std::vector<navigation::State>, navigation::Seed> path =
                    quick_reflex_planner.findPathToTarget(local_map, my_bot_location, my_target_location);
            gettimeofday(&c, NULL);
            double td = t.tv_sec + t.tv_usec / 1000000.0;
            double cd = c.tv_sec + c.tv_usec / 1000000.0;
            // std::cout<<"FPS:"<< 1/(cd-td) <<std::endl;
            // planner_quickReflex.showPath(path.first , my_bot_location, my_target_location);
            publishData(path);
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

        pub_path.publish(seed);

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

        pub_path.publish(seed);

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
}