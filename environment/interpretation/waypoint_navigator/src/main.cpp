#include <waypoint_navigator.hpp>

int main(int argc, char* argv[]) {
    WaypointNavigator waypoint_navigator;
    std::string yaw_sub_topic;
    std::string node_name = "waypoint_navigator";
    ros::init(argc, argv, std::string("waypoint_navigator"));
    ros::NodeHandle node_handle;

    node_handle.getParam(std::string("/") + node_name + std::string("/yaw_subscriber_topic"), yaw_sub_topic);
    ros::Subscriber next_waypoint_subscriber = node_handle.subscribe("waypoint_selector/next_waypoint", buffer_size, &WaypointNavigator::set_target_fix, &waypoint_navigator);
    ros::Subscriber current_fix_subscriber = node_handle.subscribe("vn_ins/fix", buffer_size, &WaypointNavigator::set_current_fix, &waypoint_navigator);
    ros::Subscriber current_yaw_subscriber = node_handle.subscribe(yaw_sub_topic.c_str(), buffer_size, &WaypointNavigator::set_heading, &waypoint_navigator);
    ros::Publisher target_publisher = node_handle.advertise<geometry_msgs::Pose2D>("waypoint_navigator/target", buffer_size);

    ros::Rate loop_rate(loop_rate_hz);
    while (ros::ok()) {
        ros::spinOnce();
        target_publisher.publish(waypoint_navigator.interpret());
        loop_rate.sleep();
    }
}