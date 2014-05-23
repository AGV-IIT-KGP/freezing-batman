#include <waypoint_selector.hpp>

int main(int argc, char* argv[]) {
    std::string node_name = "waypoint_selector"; //launchfile-> <node .... name="waypoint_selector" />
    ros::init(argc, argv, node_name);
    ros::NodeHandle node_handle;

    int strategy;
    std::string file_name;
    node_handle.getParam(std::string("/") + node_name + std::string("/filename"), file_name);
    node_handle.getParam(std::string("/") + node_name + std::string("/strategy"), strategy);
    WaypointSelector waypoint_selector(file_name, strategy);

    ros::Publisher next_waypoint_publisher = node_handle.advertise<sensor_msgs::NavSatFix>("waypoint_selector/next_waypoint", buffer_size);
    ros::Publisher nml_flag_publisher = node_handle.advertise<std_msgs::Bool>("waypoint_selector/nml_flag", buffer_size);
    ros::Subscriber planner_status_subscriber = node_handle.subscribe("local_planner/status", buffer_size, &WaypointSelector::set_planner_status, &waypoint_selector);
    ros::Subscriber fix_subscriber = node_handle.subscribe("vn_ins/fix", buffer_size, &WaypointSelector::set_current_position, &waypoint_selector);

    std_msgs::Bool nml_flag;
    ros::Rate loop_rate(loop_rate_hz);
    while (ros::ok()) {
        node_handle.getParam(std::string("/") + node_name + std::string("/proximity"), waypoint_selector.proximity_);

        next_waypoint_publisher.publish(waypoint_selector.findTarget());
        nml_flag.data = waypoint_selector.isInsideNoMansLand();
        nml_flag_publisher.publish(nml_flag);

        ros::spinOnce();
        loop_rate.sleep();
    }
}