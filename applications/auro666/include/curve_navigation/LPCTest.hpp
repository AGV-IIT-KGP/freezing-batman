/* 
 * File:   LPCTest.hpp
 * Author: samuel
 *
 * Created on 8 January, 2014, 10:37 PM
 */

#ifndef LPCTEST_HPP
#define	LPCTEST_HPP

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>

class LPCTest {
public:
    LPCTest();
    LPCTest(ros::NodeHandle node_handle);
    LPCTest(const LPCTest& orig);
    virtual ~LPCTest();
    
    void publishPose(const geometry_msgs::Pose::ConstPtr& pose_ptr);
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

    ros::Subscriber pose_subscriber;
    ros::Publisher maneuver_publisher;
    ros::Publisher pose_publisher;
    ros::Publisher map_publisher;
};

#endif	/* LPCTEST_HPP */

