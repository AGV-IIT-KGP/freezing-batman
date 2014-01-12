/* 
 * File:   UnitTester.hpp
 * Author: samuel
 *
 * Created on 25 December, 2013, 8:20 PM
 */

#ifndef UNITTESTER_HPP
#define	UNITTESTER_HPP

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>

class UnitTester {
public:
    UnitTester();
    UnitTester(ros::NodeHandle node_handle);
    UnitTester(const UnitTester& orig);
    virtual ~UnitTester();

    void sendTestData();

private:
    int map_width;
    int map_height;
    int num_samples;
    double pi;

    // msg containers for data to be sent
    geometry_msgs::PoseStamped pose_msg;
    nav_msgs::Path maneuver_msg;
    nav_msgs::OccupancyGrid map_msg;
    
    ros::Publisher maneuver_publisher;
    ros::Publisher pose_publisher;
    ros::Publisher map_publisher;
};

#endif	/* ROADNAVIGATIONTESTER_HPP */

