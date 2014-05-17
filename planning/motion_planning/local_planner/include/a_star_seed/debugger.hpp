//
//  local_planner.hpp
//  LocalPlanner
//
//  Created by Satya Prakash on 28/01/14.
//  Copyright (c) 2014 Satya Prakash. All rights reserved.
//

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


#include "/home/yash/fuerte_workspace/sandbox/freezing-batman/planning/motion_planning/local_planner/include/sa_star_seed/a_star_seed.hpp"
#include "planning/planner.hpp"
static const int MAP_MAX = 800;
static const int LOOP_RATE = 10;
static const int WAIT_TIME = 100;



namespace navigation {

    struct Pose {
        int x, y;
    };

    class Debugger {
    public:
        Debugger();
        // void debug();
        cv::Mat local_map;
        std::vector<Pose> path;
        navigation::State my_bot_location, my_target_location;
        void makeMap();
        // void publishImage();
        void showPath();
        ros::NodeHandle nh;



    private:


        ros::Subscriber sub_world_map;
        ros::Subscriber sub_bot_pose;
        ros::Subscriber sub_target_pose;
        ros::Subscriber sub_nav_msgs;


        image_transport::Publisher pub_path_image;

        const std::string pub_topic_name;
        const std::string sub_topic_name;



        image_transport::ImageTransport *it;

        void updateWorldMap(const sensor_msgs::ImageConstPtr& world_map);
        void updateNavMsg(const nav_msgs::Path&);

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


