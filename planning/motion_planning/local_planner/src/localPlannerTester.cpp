
#include "RoadNavigationTester.hpp"

int main(int argc, char **argv) {
    ros::init(argc, argv, "tester");
    ros::NodeHandle n;
    nav_msgs::Path path_msg;

    geometry_msgs::PoseWithCovarianceStamped current_pos;
    
    navigation::generate_pos(current_pos);
    navigation::selectPath(path_msg);
    ros::Publisher lanetraj_pub = n.advertise<nav_msgs::Path>("sensor_fusion/lanes/trajectory", 10);
    ros::Publisher pos_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("localization/pose", 100);
    ros::Subscriber bestpath_sub = n.subscribe("local_planner/path", 2, navigation::bestPath);

    ros::Rate loop_rate(100);
    int count = 0;
    while (ros::ok()) {
        path_msg.header.seq = count;
        path_msg.header.frame_id = count;
        path_msg.header.stamp = ros::Time::now();
        current_pos.header.seq = count;
        current_pos.header.frame_id = count;
        current_pos.header.stamp = ros::Time::now();
        pos_pub.publish(current_pos);
        lanetraj_pub.publish(path_msg);
        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }

    return 0;
}
