/* 
 * File:   GSLTest.hpp
 * Author: samuel
 *
 * Created on 24 January, 2014, 7:25 PM
 */

#ifndef GSLTEST_HPP
#define	GSLTEST_HPP

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_datatypes.h>
#include <auro666_pilot/State.h>

class GSLTest {
public:
    GSLTest();
    GSLTest(ros::NodeHandle node_handle);
    GSLTest(const GSLTest& orig);
    virtual ~GSLTest();
    
    void publishPose(const geometry_msgs::Pose::ConstPtr& pose_ptr);
    void publishState(const auro666_pilot::State::ConstPtr& state_ptr);
    void sendTestData();

private:
    int map_width;
    int map_height;

    geometry_msgs::PoseStamped pose_msg;
    nav_msgs::OccupancyGrid map_msg;

    ros::Subscriber pose_subscriber;
    ros::Subscriber state_subscriber;    
    ros::Publisher pose_publisher;
    ros::Publisher map_publisher;
    ros::Publisher state_publisher;

    void initialize();
};

#endif	/* GSLTEST_HPP */

