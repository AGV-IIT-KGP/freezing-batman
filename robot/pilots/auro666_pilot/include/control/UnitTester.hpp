/* 
 * File:   UnitTester.hpp
 * Author: samuel
 *
 * Created on 7 January, 2014, 7:30 PM
 */

#ifndef UNITTESTER_HPP
#define	UNITTESTER_HPP

#include <fstream>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Path.h>
#include <auro666_pilot/State.h>

#define PI 3.141

class UnitTester {
public:
    UnitTester();
    UnitTester(ros::NodeHandle& node_handle);
    UnitTester(const UnitTester& orig);
    virtual ~UnitTester();

    void publishPose(const geometry_msgs::Pose::ConstPtr& pose);
    void publishState(const auro666_pilot::State::ConstPtr& state);
    void selectPath();
    void sendTestData(int frame_id);

private:
    int path_size;
    int path_type;

    nav_msgs::Path path;
    
    ros::Publisher state_publisher;
    ros::Publisher pose_publisher;
    ros::Publisher path_publisher;
    ros::Subscriber state_subscriber;
    ros::Subscriber pose_subscriber;

    geometry_msgs::Pose pathGenerator(unsigned int i);
};

#endif	/* UNITTESTER_HPP */

