/* 
 * File:   waypoint_navigator.hpp
 * Author: Abinash Meher
 *
 * Created on 18 May, 2014, 7:38 AM
 */

#ifndef WAYPOINT_NAVIGATOR_HPP
#define	WAYPOINT_NAVIGATOR_HPP
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <boost/bind.hpp>
#include <cmath>

static const int buffer_size = 10;
static const int loop_rate_hz = 10;

class Waypoint_Navigator {
    sensor_msgs::NavSatFix current_gps_, target_gps_;
public:
    void setTargetGPS(sensor_msgs::NavSatFix target);
    void setCurrentGPS(sensor_msgs::NavSatFix current);
    geometry_msgs::Pose2D interpret();
};

#endif	/* WAYPOINT_NAVIGATOR_HPP */

