//
//  local_planner.hpp
//  LocalPlanner
//
//  Created by Satya Prakash on 28/01/14.
//  Copyright (c) 2014 Satya Prakash. All rights reserved.
//

#include <cmath>
#include <sstream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/String.h>
#include <a_star_seed/a_star_seed.hpp>

namespace navigation {

    struct Pose {
        int x, y;
    };

    class Debugger {
    public:
        Debugger();
        cv::Mat local_map;
        std::vector<Pose> path;
        navigation::State bot_pose, target_pose;
        void makeMap();
        void showPath();
        ros::NodeHandle node_handle;

    private:
        int map_max_cols, map_max_rows;
        bool load_in_planner;
        ros::Subscriber fusion_map_subscriber;
        ros::Subscriber target_subscriber;
        ros::Subscriber path_subscriber;
        ros::Subscriber status_subscriber;

        void updateFusionMap(const sensor_msgs::ImageConstPtr& fusion_map);
        void updatePath(const nav_msgs::Path& path);
        void updateStatus(const std_msgs::String status);

        inline void updateTargetPose(const geometry_msgs::Pose2D _pose) {
            int x = _pose.x;
            int y = _pose.y;
            int theta = (_pose.theta)*180 / M_PI;
            target_pose = navigation::State(x, y, theta, 0);
        }
    };
}

