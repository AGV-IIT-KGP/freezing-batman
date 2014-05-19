/* 
 * File:   waypoint_navigator.hpp
 * Author: Abinash Meher
 *
 * Created on 18 May, 2014, 7:38 AM
 */

#ifndef WAYPOINT_NAVIGATOR_HPP
#define	WAYPOINT_NAVIGATOR_HPP

#include <cmath>
#include <boost/bind.hpp>
#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

static const int buffer_size = 10;
static const int loop_rate_hz = 10;

class WaypointNavigator {
private:
    float heading_;
    sensor_msgs::NavSatFix current_fix_, target_fix_;

public:
    void set_target_fix(sensor_msgs::NavSatFix target_fix);
    void set_current_fix(sensor_msgs::NavSatFix current_fix);
    void set_heading(std_msgs::Float64 heading);
    geometry_msgs::Pose2D interpret();
};

#endif	/* WAYPOINT_NAVIGATOR_HPP */

