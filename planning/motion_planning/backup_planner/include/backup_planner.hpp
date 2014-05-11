#ifndef __BACKUPPLANNER__BACKUPPLANNER__
#define __BACKUPPLANNER__BACKUPPLANNER__

#include <sys/time.h>
#include <sstream>
#include <iostream>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv/cvaux.hpp>
#include <opencv/cxcore.h>
#include <cv.h>
#include <highgui.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"

#include "a_star_seed/a_star_seed.hpp"
#include "a_star_seed/state.hpp"
#include "a_star_seed/state_of_car.hpp"
#include "a_star_seed/ss_priority_queue.hpp"
#include "planning/planner.hpp"

namespace navigation{

	class backupPlanner{
	public:
        backupPlanner(ros::NodeHandle& nodehandle);
        void plan();
    private:
        ros::NodeHandle nh;
        
        ros::Subscriber sub_world_map;
        ros::Subscriber sub_bot_pose;
        ros::Subscriber sub_target_pose;
        
        ros::Publisher pub_path;
        
        const std::string pub_topic_name;
        const std::string sub_topic_name;

        navigation::State my_bot_location, my_target_location;

        cv::Mat local_map;

        const image_transport::ImageTransport *it;

        void updateWorldMap(const sensor_msgs::ImageConstPtr& world_map);
        inline void updateBotPose(const geometry_msgs::Pose::ConstPtr _pose);
        inline void updateTargetPose(const geometry_msgs::Pose::ConstPtr _pose);
        void publishData(std::pair<std::vector<navigation::StateOfCar>, navigation::Seed>& path);
	};
}

#endif /* defined(__LOCALPLANNER__LOCALPLANNER__) */
