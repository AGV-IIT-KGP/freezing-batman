/* 
 * File:   Debugger.hpp
 * Author: auro666
 *
 * Created on 11 January, 2014, 9:39 PM
 */

#ifndef DEBUGGER_HPP
#define	DEBUGGER_HPP

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

class Debugger {
public:
    Debugger();
    Debugger(ros::NodeHandle node_handle);
    Debugger(const Debugger& orig);
    virtual ~Debugger();

    void SetCurrent_pose(geometry_msgs::PoseStamped current_pose) {
        this->current_pose = current_pose.pose;
    }

    void SetManeuver(nav_msgs::Path maneuver) {
        this->maneuver = maneuver; // Meters
    }

    void SetPath(nav_msgs::Path path) {
        this->path = path;
    }

    void display(int debug_mode);

private:
    int map_width;
    int map_height;
    std::string window_name;
    double scale;
    cv::Mat image;

    geometry_msgs::Pose current_pose;
    nav_msgs::Path path;
    nav_msgs::Path maneuver;

    ros::Subscriber pose_subscriber;
    ros::Subscriber maneuver_subscriber;
    ros::Subscriber path_subscriber;
};

#endif	/* DEBUGGER_HPP */

