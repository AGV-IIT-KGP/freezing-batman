//
//  local_planner.hpp
//  LocalPlanner
//
//  Created by Satya Prakash on 28/01/14.
//  Copyright (c) 2014 Satya Prakash. All rights reserved.
//
#ifndef __LOCALPLANNER__LOCALPLANNER__
#define __LOCALPLANNER__LOCALPLANNER__

#include <cmath>
#include <sstream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <cv_bridge/CvBridge.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <image_transport/image_transport.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/String.h>
#include <a_star_seed/a_star_seed.hpp>
#include <local_planner/Seed.h>
#include <planning/planner.hpp>

namespace navigation {

    class LocalPlanner : public planning::Planner {
    public:
        int status;
        LocalPlanner(ros::NodeHandle& nodehandle);
        void planWithAstarSeed(navigation::AStarSeed& astar_seed_planner);
        void planWithQuickReflex(navigation::quickReflex& quick_reflex_planner);
        int planning_strategy_;

    private:
        int map_max_cols, map_max_rows;
        ros::NodeHandle node_handle;


        ros::Subscriber fusion_map_subscriber;
        ros::Subscriber target_subscriber;
        ros::Subscriber planning_strategy_subscriber;

        ros::Publisher seed_publisher;
        ros::Publisher path_publisher;
        ros::Publisher status_publisher;
        image_transport::Publisher pub_path_image;

        navigation::State bot_pose, target_pose;
        cv::Mat local_map;

        void publishData(std::pair<std::vector<navigation::StateOfCar>, navigation::Seed>& path);
        void publishData(std::pair<std::vector<navigation::State>, navigation::Seed>& path);
        void publishImage(cv::Mat image);
        void publishStatusQuickReflex(int status);
        void publishStatusAStarSeed(int status);
        void updateFusionMap(const sensor_msgs::ImageConstPtr& fusion_map);
        void updateStrategy(const std_msgs::String planner_strategy);

        inline void updateTargetPose(const geometry_msgs::Pose2D::ConstPtr _pose) {
            int x = _pose->x;
            int y = _pose->y;
            int theta = (_pose->theta)*180 / M_PI;
            target_pose = navigation::State(x, y, theta, 0);
        }
    };
}

#endif /* defined(__LOCALPLANNER__LOCALPLANNER__) */
