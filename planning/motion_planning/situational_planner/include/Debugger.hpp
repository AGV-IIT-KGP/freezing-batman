/* 
 * File:   Debugger.hpp
 * Author: samuel
 *
 * Created on 24 January, 2014, 3:25 AM
 */

#ifndef DEBUGGER_HPP
#define	DEBUGGER_HPP

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

class Debugger {
public:
    Debugger();
    Debugger(ros::NodeHandle node_handle);
    Debugger(const Debugger& orig);
    virtual ~Debugger();

    void setCurrent_pose(geometry_msgs::PoseStamped current_pose) {
        this->current_pose = current_pose.pose;
    }

    void setManeuver(nav_msgs::Path maneuver) {
        this->maneuver = maneuver;
    }

    void display(int debug_mode);

private:
    int map_width;
    int map_height;
    std::string window_name;
    double scale;
    cv::Mat image;

    geometry_msgs::Pose current_pose;
    nav_msgs::Path maneuver;

    ros::Subscriber pose_subscriber;
    ros::Subscriber maneuver_subscriber;

    void initialize();
};

#endif	/* DEBUGGER_HPP */

