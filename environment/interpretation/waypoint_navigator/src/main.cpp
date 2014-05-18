#include "waypoint_navigator/waypoint_navigator.hpp"

int main(int argc, char* argv[]) {
    Waypoint_Navigator waypoint_navigator;

    ros::init(argc, argv, std::string("waypoint_navigator"));
    ros::NodeHandle nh;

    ros::Subscriber sub_next_waypoint = nh.subscribe("waypoint_selector/next_waypoint", buffer_size, &Waypoint_Navigator::setTargetGPS, &waypoint_navigator);
    ros::Subscriber sub_current_gps = nh.subscribe("vn_ins/fix", buffer_size, &Waypoint_Navigator::setCurrentGPS, &waypoint_navigator);
    ros::Publisher pub_relative_pose = nh.advertise<geometry_msgs::Pose2D>("waypoint_navigator/proposed_target", buffer_size);

    ros::Rate loop_rate(loop_rate_hz);
    while (ros::ok()) {
        ros::spinOnce();
        pub_relative_pose.publish(waypoint_navigator.interpret());
        loop_rate.sleep();
    }
}