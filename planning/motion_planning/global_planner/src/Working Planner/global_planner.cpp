#include <OnlineMapsPlanner.hpp>
#include <ros/ros.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "global_planner");
    ros::NodeHandle node_handle;

    ros::Publisher waypoint_publisher = node_handle.advertise<nav_msgs::Path>("global_planner/waypoints", 10);

    OnlineMapsPlanner online_maps_planner("../data/sim_points.txt");

    ros::Rate loop_rate(1);
    while (ros::ok()) {
        waypoint_publisher.publish(online_maps_planner.getWaypoints());
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

