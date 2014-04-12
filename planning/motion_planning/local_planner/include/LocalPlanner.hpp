/* 
 * File:   LocalPlanner.hpp
 * Author: samuel
 *
 * Created on 25 January, 2014, 10:37 PM
 */

#ifndef LOCALPLANNER_HPP
#define	LOCALPLANNER_HPP

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>

class LocalPlanner {
public:
    LocalPlanner();
    LocalPlanner(ros::NodeHandle node_handle);
    LocalPlanner(const LocalPlanner& orig);
    virtual ~LocalPlanner();

    void SetManeuver(nav_msgs::Path maneuver) {
        this->maneuver = maneuver;
    }

    void SetMap(nav_msgs::OccupancyGrid map) {
        this->map = map;
    }

    void SetPose(geometry_msgs::PoseStamped pose) {
        this->pose = pose;
    }

    void plan();

private:
    double scale;
    
    nav_msgs::Path maneuver;
    geometry_msgs::PoseStamped pose;
    nav_msgs::OccupancyGrid map;

    ros::Subscriber maneuver_subscriber;
    ros::Subscriber pose_subscriber;
    ros::Subscriber map_subscriber;
    ros::Publisher path_publisher;
};

#endif	/* LOCALPLANNER_HPP */

