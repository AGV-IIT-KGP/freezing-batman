/* 
 * File:   LocalPlanner.cpp
 * Author: samuel
 * 
 * Created on 25 January, 2014, 10:37 PM
 */

#include <LocalPlanner.hpp>
#include <road_navigation/RoadNavigation.hpp>

LocalPlanner::LocalPlanner() {
}

LocalPlanner::LocalPlanner(ros::NodeHandle node_handle) {
    scale = .01;

    maneuver_subscriber = node_handle.subscribe("situational_planner/maneuver", 2, &LocalPlanner::SetManeuver, this);
    pose_subscriber = node_handle.subscribe("localization/pose", 2, &LocalPlanner::SetPose, this);
    map_subscriber = node_handle.subscribe("environment/map", 2, &LocalPlanner::SetMap, this);
    path_publisher = node_handle.advertise<nav_msgs::Path>("local_planner/path", 10);
}

LocalPlanner::LocalPlanner(const LocalPlanner& orig) {
}

LocalPlanner::~LocalPlanner() {
}

void LocalPlanner::plan() {
    navigation::RoadNavigation planner;
    nav_msgs::Path path = planner.plan(maneuver, pose, map);
    for (unsigned int pose_id = 0; pose_id < path.poses.size(); pose_id++) {
        path.poses.at(pose_id).pose.position.x *= scale; // Meters
        path.poses.at(pose_id).pose.position.y *= scale;
    }
    path_publisher.publish(path);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "local_planner");
    ros::NodeHandle node_handle;
    LocalPlanner local_planner(node_handle);

    ros::Rate loop_rate(10);
    while (ros::ok()) {
        ROS_INFO("How's that for a plan?");
        
        local_planner.plan();
        ros::spinOnce();
        loop_rate.sleep();
    }
}