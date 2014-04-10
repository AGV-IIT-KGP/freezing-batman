#include <UnitTester.hpp>
#include <std_msgs/Header.h>
#include <geometry_msgs/PoseStamped.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "local_planner_tester");
    ros::NodeHandle node_handle;
    UnitTester tester;

    ros::Publisher lane_trajectory_publisher = node_handle.advertise<nav_msgs::Path>("sensor_fusion/lanes/trajectory", 10);
    ros::Publisher current_pose_publisher = node_handle.advertise<geometry_msgs::PoseStamped>("localization/pose", 100);
    ros::Publisher map_publisher = node_handle.advertise<nav_msgs::OccupancyGrid>("slam/map", 100);
    ros::Subscriber path_subscriber = node_handle.subscribe("local_planner/path", 2, &UnitTester::SetPath, &tester);

    geometry_msgs::PoseStamped current_pose;
    nav_msgs::OccupancyGrid map;
    nav_msgs::Path lane_trajectory;
    std_msgs::Header header;
    ros::Rate loop_rate(100);
    int count = 0;
    while (ros::ok()) {
        header.seq = count;
        header.frame_id = count;
        header.stamp = ros::Time::now();

        current_pose.pose = tester.GetCurrent_pose();
        current_pose.header = header;
        current_pose_publisher.publish(current_pose);

        lane_trajectory = tester.GetLane_trajectory();
        lane_trajectory.header = header;
        lane_trajectory_publisher.publish(lane_trajectory);

        map = tester.GetMap();
        map.header = header;
        map_publisher.publish(map);
        
        ros::spinOnce();
        loop_rate.sleep();
        ++count;

        tester.display();
    }

    return 0;
}