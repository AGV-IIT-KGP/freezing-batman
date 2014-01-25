#include <SituationSolver.hpp>
#include <ros/ros.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "situational_planner");
    ros::NodeHandle node_handle;

    SituationSolver solver;

    ros::Subscriber pose_subscriber = node_handle.subscribe("localization/pose", 2, &SituationSolver::SetCurrent_pose, &solver);
    ros::Subscriber waypoint_subscriber = node_handle.subscribe("global_planner/waypoints", 2, &SituationSolver::SetWaypoints, &solver);
    
    ros::Publisher maneuver_publisher = node_handle.advertise<nav_msgs::Path>("situational_planner/maneuver", 1);

    ros::Rate loop_rate(1);
    int seq_id = 0;
    while (ros::ok()) {
        nav_msgs::Path maneuver = solver.nextManeuver();
        maneuver.header.stamp = ros::Time::now();        
        maneuver.header.seq = seq_id++;
        maneuver_publisher.publish(maneuver);
        
        ros::spinOnce();
        loop_rate.sleep();
    }
}

