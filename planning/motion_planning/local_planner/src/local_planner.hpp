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
#include <cmath>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <nav_msgs/Path.h>


#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose2D.h"

#include "local_planner/Seed.h"
#include "a_star_seed/a_star_seed.hpp"
#include "planning/planner.hpp"

static const int WAIT_TIME = 100;



namespace navigation {

    class LocalPlanner : public planning::Planner {
    public:
        LocalPlanner(ros::NodeHandle& nodehandle);
        void planWithAstarSeed();
        void planWithQuickReflex();
        int status;
    private:
        ros::NodeHandle nh;
        int planning_strategy;
        void loadParams(ros::NodeHandle& nh);


        ros::Subscriber sub_world_map;
        ros::Subscriber sub_bot_pose;
        ros::Subscriber sub_target_pose;

        ros::Publisher pub_seed;
        ros::Publisher pub_nav_msgs;
        ros::Publisher pub_status;
        image_transport::Publisher pub_path_image;
        

        const std::string pub_topic_name;
        const std::string sub_topic_name;

        navigation::State my_bot_location, my_target_location;
        cv::Mat local_map;
        image_transport::ImageTransport *it;

        void updateWorldMap(const sensor_msgs::ImageConstPtr& world_map);
        void publishData(std::pair<std::vector<navigation::StateOfCar>, navigation::Seed>& path);
        void publishData(std::pair<std::vector<navigation::State>, navigation::Seed>& path);
        void publishImage(cv::Mat image);
        void publishStatusQuickReflex(int status);
        void publishStatusAStarSeed(int status);


        inline void updateTargetPose(const geometry_msgs::Pose2D::ConstPtr _pose) {
            int x = _pose->x;
            int y = _pose->y;
            int theta = (_pose->theta)*180/M_PI;
            my_target_location = navigation::State(x, y, theta, 0);
        }
    };
}


#endif /* defined(__LOCALPLANNER__LOCALPLANNER__) */
