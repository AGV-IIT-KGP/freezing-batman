//
//  local_planner.hpp
//  LocalPlanner
//
//  Created by Satya Prakash on 28/01/14.
//  Copyright (c) 2014 Satya Prakash. All rights reserved.
//
#ifndef __LOCALPLANNER__LOCALPLANNER__
#define __LOCALPLANNER__LOCALPLANNER__

#include <sys/time.h>
#include <sstream>
#include <iostream>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv/cvaux.hpp>
#include <opencv/cxcore.h>
#include <cv.h>
#include <highgui.h>
#include <cv_bridge/CvBridge.h>
#include <opencv/cvwimage.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <nav_msgs/Path.h>


#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"

#include "local_planner/Seed.h"
#include "a_star_seed/a_star_seed.hpp"
#include "planning/planner.hpp"
static const int MAP_MAX = 800;
static const int LOOP_RATE = 10;
static const int WAIT_TIME = 100;



namespace navigation {

    class LocalPlanner : public planning::Planner {
    public:
        LocalPlanner(ros::NodeHandle& nodehandle);
        void plan();
        void planWithQuickReflex();
    private:
        ros::NodeHandle node_handle;

        ros::Subscriber sub_world_map;
        ros::Subscriber sub_bot_pose;
        ros::Subscriber sub_target_pose;

        ros::Publisher pub_path;
        image_transport::Publisher pub_path_image;
        ros::Publisher pub_nav_msgs;

        const std::string pub_topic_name;
        const std::string sub_topic_name;

        navigation::State my_bot_location, my_target_location;

        cv::Mat local_map;

        image_transport::ImageTransport *it;


        void updateWorldMap(const sensor_msgs::ImageConstPtr& world_map);
        void publishData(std::pair<std::vector<navigation::StateOfCar>, navigation::Seed>& path);
        void publishData(std::pair<std::vector<navigation::State>, navigation::Seed>& path);
        void publishImage(cv::Mat image);

        inline void updateBotPose(const geometry_msgs::Pose::ConstPtr _pose) {
            int x = _pose->position.x;
            int y = _pose->position.y;
            int z = _pose->position.z;
            my_bot_location = navigation::State(x, y, z, 0);
        }

        inline void updateTargetPose(const geometry_msgs::Pose::ConstPtr _pose) {
            int x = _pose->position.x;
            int y = _pose->position.y;
            int z = _pose->position.z;
            my_target_location = navigation::State(x, y, z, 0);
        }
    };
}


#endif /* defined(__LOCALPLANNER__LOCALPLANNER__) */
