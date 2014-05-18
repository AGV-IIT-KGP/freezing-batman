#include <waypoint_selector/waypoint_selector.hpp>

int main(int argc, char* argv[]) {
    int strategy;
    std::string filename;
    sensor_msgs::NavSatFix target;
    bool nml;
    std_msgs::Bool std_nml;
    std::string node_name = "waypoint_selector"; //launchfile-> <node .... name="waypoint_selector" />

    ros::NodeHandle nh;

    ros::init(argc, argv, std::string("waypoint_selector"));
    nh.getParam(std::string("/") + node_name + std::string("/proximity"), proximity);
    nh.getParam(std::string("/") + node_name + std::string("/filename"), filename);
    nh.getParam(std::string("/") + node_name + std::string("/strategy"), strategy);

    Waypoint_Selector waypoint_selector(filename, strategy);

    ros::Publisher pub_selected_waypoint = nh.advertise<sensor_msgs::NavSatFix>("waypoint_selector/next_waypoint", buffer_size);
    ros::Publisher pub_nml_flag = nh.advertise<std_msgs::Bool>("waypoint_selector/nml_flag", buffer_size);
    ros::Subscriber sub_planner_status = nh.subscribe("local_planner/status", buffer_size, &Waypoint_Selector::setPlannerStatus, &waypoint_selector);
    ros::Subscriber sub_current_position = nh.subscribe("vn_ins/fix", buffer_size, &Waypoint_Selector::setCurrentPosition, &waypoint_selector);

    ros::Rate loop_rate(loop_rate_hz);
    while (ros::ok()) {
        ros::spinOnce();
        target = waypoint_selector.getTarget();
        nml = waypoint_selector.ifnml();
        std_nml.data = nml;
        pub_nml_flag.publish(std_nml);
        pub_selected_waypoint.publish(target);
        loop_rate.sleep();
    }

}