#include <waypoint_selector/waypoint_selector.hpp>

int main(int argc, char* argv[]) {
    int strategy;
    std::string filename;
    sensor_msgs::NavSatFix target;
    bool nml;
    std_msgs::Bool std_nml;
    std::string node_name = "waypoint_selector"; //launchfile-> <node .... name="waypoint_selector" />

    ros::init(argc, argv, std::string("waypoint_selector"));
    ros::NodeHandle node_handle;

    node_handle.getParam(std::string("/") + node_name + std::string("/proximity"), proximity);
    node_handle.getParam(std::string("/") + node_name + std::string("/filename"), filename);
    node_handle.getParam(std::string("/") + node_name + std::string("/strategy"), strategy);

    Waypoint_Selector waypoint_selector(filename, strategy);

    ros::Publisher next_waypoint_publisher = node_handle.advertise<sensor_msgs::NavSatFix>("waypoint_selector/next_waypoint", buffer_size);
    ros::Publisher nml_flag_publisher = node_handle.advertise<std_msgs::Bool>("waypoint_selector/nml_flag", buffer_size);
    ros::Subscriber planner_status_subscriber = node_handle.subscribe("local_planner/status", buffer_size, &Waypoint_Selector::setPlannerStatus, &waypoint_selector);
    ros::Subscriber fix_subscriber = node_handle.subscribe("vn_ins/fix", buffer_size, &Waypoint_Selector::setCurrentPosition, &waypoint_selector);

    ros::Rate loop_rate(loop_rate_hz);
    while (ros::ok()) {
        ros::spinOnce();
        target = waypoint_selector.getTarget();
        nml = waypoint_selector.ifnml();
        std_nml.data = nml;
        nml_flag_publisher.publish(std_nml);
        next_waypoint_publisher.publish(target);
        loop_rate.sleep();
    }
}