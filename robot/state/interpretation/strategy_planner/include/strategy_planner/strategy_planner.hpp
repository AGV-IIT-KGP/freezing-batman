#include <iostream>
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <iostream>
#include <cstdlib>
#include <signal.h>
#include <message_filters/subscriber.h>
#include <vector>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include "geometry_msgs/Pose.h"
#include <boost/bind.hpp>

enum Navigators {
	Dummy_Navigator    	= 0,
	Nose_Navigator     	= 1,
    Waypoint_Navigator 	= 2,
    Lane_Navigator     	= 3,
};

enum Planners {
	A_Star_Seed 	= 0,
	Quick_Response 	= 1,
};

geometry_msgs::Pose dummy_target_, nose_target_, waypoint_target_, lane_target_;

void setDummyTarget(geometry_msgs proposed_dummy_target_);
void setNoseTarget(geometry_msgs proposed_nose_target_);
void setWaypointTarget(geometry_msgs proposed_waypoint_target_);
void setLaneTarget(geometry_msgs proposed_lane_target_);

inline geometry_msgs::Pose getDummyTarget() 	const { return dummy_target_; }
inline geometry_msgs::Pose getNoseTarget() 		const { return nose_target_; }
inline geometry_msgs::Pose getWaypointTarget() 	const { return waypoint_target_; }
inline geometry_msgs::Pose getLaneTarget() 		const { return lane_target_; }
