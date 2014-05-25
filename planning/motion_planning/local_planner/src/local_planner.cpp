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
        map_max_rows = 1000;
        map_max_cols = 1000;
        node_handle.getParam("local_planner/map_max_rows", map_max_rows);
        node_handle.getParam("local_planner/map_max_cols", map_max_cols);
        local_map = cv::Mat::zeros(map_max_rows, map_max_cols, CV_8UC1);
        bot_pose = navigation::State(map_max_cols / 2, map_max_rows / 10, 90, 0);
        target_pose = navigation::State(map_max_cols / 2, map_max_rows * .95, 90, 0);

        fusion_map_subscriber = node_handle.subscribe("/data_fuser/map", 10, &LocalPlanner::updateFusionMap, this);
        target_subscriber = node_handle.subscribe("strategy_planner/target", 10, &LocalPlanner::updateTargetPose, this);
        planning_strategy_subscriber = node_handle.subscribe("strategy_planner/which_planner", 10, &LocalPlanner::updateStrategy, this);

        seed_publisher = node_handle.advertise<local_planner::Seed>("local_planner/seed", 1000);
        status_publisher = node_handle.advertise<std_msgs::String>("local_planner/status", 1000);
        path_publisher = node_handle.advertise<nav_msgs::Path>("local_planner/path", 10);
        truncated_target_publisher = node_handle.advertise<geometry_msgs::Pose2D>("local_planner/target", 1000);
        image_transport::ImageTransport it(node_handle);
        image_publisher = it.advertise("local_planner/map", 1000);
    }

    void LocalPlanner::updateFusionMap(const sensor_msgs::ImageConstPtr& world_map) {
        //TODO : copy function for occupancy grid
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(world_map, sensor_msgs::image_encodings::MONO8);
            local_map = cv_ptr->image;
            cv::rectangle(local_map, cv::Point(0 * local_map.cols, 0 * local_map.rows), cv::Point(.2 * local_map.cols, 1 * local_map.rows), cv::Scalar(0, 0, 0), CV_FILLED, 8, 0);
            cv::rectangle(local_map, cv::Point(.2 * local_map.cols, 1 * local_map.rows), cv::Point(1 * local_map.cols, .8 * local_map.rows), cv::Scalar(0, 0, 0), CV_FILLED, 8, 0);
            cv::rectangle(local_map, cv::Point(.8 * local_map.cols, .8 * local_map.rows), cv::Point(1 * local_map.cols, 0 * local_map.rows), cv::Scalar(0, 0, 0), CV_FILLED, 8, 0);
            cv::rectangle(local_map, cv::Point(.2 * local_map.cols, 0 * local_map.rows), cv::Point(.8 * local_map.cols, .2 * local_map.rows), cv::Scalar(0, 0, 0), CV_FILLED, 8, 0);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
    }

    void LocalPlanner::updateStrategy(const std_msgs::String planner_strategy) {
        if (planner_strategy.data == std::string("A_Star_Seed")) {
            planning_strategy_ = 0;
        } else if (planner_strategy.data == std::string("Quick_Reflex")) {
            planning_strategy_ = 1;
        }
    }

    void LocalPlanner::planWithAstarSeed(navigation::AStarSeed& astar_seed_planner) {
        // struct timeval t, c;
        // gettimeofday(&t, NULL);
        std::pair<std::vector<navigation::StateOfCar>, navigation::Seed> path = astar_seed_planner.findPathToTarget(local_map, bot_pose, target_pose, astar_seed_planner.distance_transform, astar_seed_planner.debug_current_state, astar_seed_planner.status);
        // gettimeofday(&c, NULL);
        // double td = t.tv_sec + t.tv_usec / 1000000.0;
        // double cd = c.tv_sec + c.tv_usec / 1000000.0;

        publishData(path);
        publishStatusAStarSeed(astar_seed_planner.status);
        truncated_target_publisher.publish(truncated_target_pose);
        publishImage(local_map);
    }

    void LocalPlanner::planWithQuickReflex(navigation::quickReflex& quick_reflex_planner) {
        // struct timeval t, c;
        // gettimeofday(&t, NULL);
        std::pair<std::vector<navigation::State>, navigation::Seed> path = quick_reflex_planner.findPathToTarget(local_map, bot_pose, target_pose, quick_reflex_planner.status);
        // gettimeofday(&c, NULL);
        // double td = t.tv_sec + t.tv_usec / 1000000.0;
        // double cd = c.tv_sec + c.tv_usec / 1000000.0;
        // std::cout<<"FPS:"<< 1/(cd-td) <<std::endl;
        // planner_quickReflex.showPath(path.first , my_bot_location, my_target_location);
        publishData(path);
        publishStatusQuickReflex(quick_reflex_planner.status);
        truncated_target_publisher.publish(truncated_target_pose);
        publishImage(local_map);
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

    void LocalPlanner::publishImage(cv::Mat& image) {
        cv_bridge::CvImage out_msg;
        out_msg.encoding = sensor_msgs::image_encodings::MONO8;
        out_msg.image = image;
        image_publisher.publish(out_msg.toImageMsg());
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

    void LocalPlanner::truncate(int& pose_target_x, int& pose_target_y) {
        int pose_x = pose_target_x;
        int pose_y = pose_target_y;
        if ((pose_y <= 0.9 * map_max_rows && pose_y >= 0.1 * map_max_rows) && (pose_x <= 0.9 * map_max_cols && pose_x >= 0.1 * map_max_cols)) {
            if (local_map.at<uchar>(pose_y, pose_x) < 225) {
                return;
            } else {
                srand(time(NULL));
                int bracket = 200;
                while (true) {
                    int count = 0;
                    for (int i = 0; i < 10; ++i) {
                        int temp_x = rand() % bracket - bracket / 2;
                        int temp_y = rand() % bracket - bracket / 2;

                        temp_x = temp_x + pose_x;
                        temp_y = temp_y + pose_y;
                        if (local_map.at<uchar>(temp_y, temp_x) >= 225) {
                            count++;
                        }
                        if (count > 2) {
                            pose_target_x = temp_x;
                            pose_target_y = temp_y;
                            return;
                        }
                    }
                    bracket += 100;
                    if (bracket >= 600) {
                        for (int j = pose_y;; --j) {
                            if (local_map.at<uchar>(j, pose_x) <= 225) {
                                pose_target_y = j;
                                pose_target_x = pose_x;
                                return;
                            }
                        }
                    }
                }
            }
        }

        double pose_target_x_on_boundary, pose_target_y_on_boundary;
        int pose_bot_x = .5 * map_max_cols;
        int pose_bot_y = .1 * map_max_rows;

        if (pose_y < 0.1 * map_max_rows) {
            pose_target_y = 0.05 * map_max_rows;
            pose_target_x = (pose_x < pose_bot_x) ? 0.25 * map_max_cols : 0.75 * map_max_cols;
            return;
        }

        // L1: y = 0.9A
        pose_target_x_on_boundary = pose_bot_x + (0.9 * map_max_cols - pose_bot_y) * (pose_x - pose_bot_x) / (pose_y - pose_bot_y);
        pose_target_y_on_boundary = 0.9 * map_max_rows;
        if (((0.1 * map_max_cols <= pose_target_x_on_boundary) && (pose_target_x_on_boundary <= 0.9 * map_max_cols)) && ((pose_bot_y - 0.9 * map_max_rows) * (pose_y - 0.9 * map_max_rows) <= 0) && (pose_y >= 0.9 * map_max_rows)) {
            pose_target_x = pose_target_x_on_boundary;
            pose_target_y = pose_target_y_on_boundary;
            return;
        }

        // L2: y = 0.1A
        pose_target_x_on_boundary = pose_bot_x + (0.1 * map_max_rows - pose_bot_y) * (pose_x - pose_bot_x) / (pose_y - pose_bot_y);
        pose_target_y_on_boundary = 0.1 * map_max_rows;

        if (((0.1 * map_max_rows <= pose_target_x_on_boundary) && (pose_target_x_on_boundary <= 0.9 * map_max_cols)) && ((pose_bot_y - 0.1 * map_max_rows) * (pose_y - 0.1 * map_max_rows) <= 0) && (pose_y <= 0.1 * map_max_rows)) {
            pose_target_x = pose_target_x_on_boundary;
            pose_target_y = pose_target_y_on_boundary;
            return;
        }

        // L3: x = 0.1A
        pose_target_x_on_boundary = 0.1 * map_max_cols;
        pose_target_y_on_boundary = pose_bot_y + (0.1 * map_max_rows - pose_bot_x) * (pose_y - pose_bot_y) / (pose_x - pose_bot_x);
        if (pose_target_y_on_boundary > .1 * map_max_cols && pose_target_y_on_boundary < .9 * map_max_rows && (pose_x < 0.1 * map_max_cols)) {
            pose_target_x = pose_target_x_on_boundary;
            pose_target_y = pose_target_y_on_boundary;
            return;
        }

        // L4: x = 0.9A
        pose_target_x_on_boundary = 0.9 * map_max_cols;
        pose_target_y_on_boundary = pose_bot_y + (0.9 * map_max_rows - pose_bot_x) * (pose_y - pose_bot_y) / (pose_x - pose_bot_x);
        if (pose_target_y_on_boundary > .1 * map_max_rows && pose_target_y_on_boundary < .9 * map_max_rows && pose_x > 0.9 * map_max_cols) {
            pose_target_x = pose_target_x_on_boundary;
            pose_target_y = pose_target_y_on_boundary;
            return;
        }
    }
}